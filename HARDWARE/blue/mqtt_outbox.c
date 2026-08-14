/**
 * @file mqtt_outbox.c
 * @brief MQTT 单写线程、双优先级消息队列和 RAM 断线补传。
 *
 * 所有 USART3 上行发布都集中到 mqtttx 线程，防止多个业务线程交叉写入
 * ESP8266。关键消息优先；离线遥测使用固定容量循环队列保存，不使用堆。
 */
#include "mqtt_outbox.h"
#include "cloud_service.h"
#include "usart3.h"
#include <rtthread.h>
#include <stdio.h>
#include <string.h>

#define MQTT_CRITICAL_QUEUE_DEPTH 8
#define MQTT_NORMAL_QUEUE_DEPTH   16
#define MQTT_THREAD_STACK_SIZE    1024
#define MQTT_THREAD_PRIORITY      7
#define MQTT_OFFLINE_QUEUE_DEPTH  MQTT_OFFLINE_TELEMETRY_CAPACITY

typedef enum
{
    MQTT_OUTBOX_COMMAND = 0,  /* 已编码为完整 AT 命令。 */
    MQTT_OUTBOX_TELEMETRY = 1 /* 尚未编码的结构化遥测。 */
} mqtt_outbox_item_type_t;

typedef struct
{
    mqtt_outbox_item_type_t type;
    u16 length;
    char command[MQTT_OUTBOX_COMMAND_MAX]; /* command 类型的有效负载。 */
    gateway_state_snapshot_t telemetry;
    u8 ble_online;
} mqtt_outbox_item_t;

typedef struct
{
    gateway_state_snapshot_t telemetry;
    u8 ble_online;
    rt_tick_t captured_tick; /* 转入离线队列的系统节拍，用于计算缓存时长。 */
} mqtt_offline_telemetry_t;

static rt_mq_t mqtt_critical_mq;
static rt_mq_t mqtt_normal_mq;
static rt_thread_t mqtt_thread;
static volatile u32 mqtt_critical_drops;
static volatile u32 mqtt_normal_drops;
static mqtt_offline_telemetry_t mqtt_offline_queue[MQTT_OFFLINE_QUEUE_DEPTH];
static u16 mqtt_offline_head;
static u16 mqtt_offline_tail;
static u16 mqtt_offline_count;
static volatile u32 mqtt_offline_drops;

static u8 mqtt_offline_enqueue(const mqtt_outbox_item_t *item)
{
    u8 result = 0;
    if (item == RT_NULL || item->type != MQTT_OUTBOX_TELEMETRY) return 0;
    /* display 线程会并发读取计数，使用短临界区保护 head/tail/count。 */
    rt_enter_critical();
    if (mqtt_offline_count < MQTT_OFFLINE_QUEUE_DEPTH)
    {
        mqtt_offline_queue[mqtt_offline_tail].telemetry = item->telemetry;
        mqtt_offline_queue[mqtt_offline_tail].ble_online = item->ble_online;
        mqtt_offline_queue[mqtt_offline_tail].captured_tick = rt_tick_get();
        mqtt_offline_tail = (u16)((mqtt_offline_tail + 1U) % MQTT_OFFLINE_QUEUE_DEPTH);
        ++mqtt_offline_count;
        result = 1;
    }
    else
    {
        /* 满时保留较早数据并丢弃新数据，便于恢复后按时间顺序补传。 */
        ++mqtt_offline_drops;
    }
    rt_exit_critical();
    return result;
}

static u8 mqtt_offline_dequeue(mqtt_offline_telemetry_t *item)
{
    u8 result = 0;
    if (item == RT_NULL) return 0;
    rt_enter_critical();
    if (mqtt_offline_count)
    {
        /* FIFO 出队保证补传顺序与采集顺序一致。 */
        *item = mqtt_offline_queue[mqtt_offline_head];
        mqtt_offline_head = (u16)((mqtt_offline_head + 1U) % MQTT_OFFLINE_QUEUE_DEPTH);
        --mqtt_offline_count;
        result = 1;
    }
    rt_exit_critical();
    return result;
}

static u8 mqtt_make_telemetry_command(const gateway_state_snapshot_t *snapshot,
                                      u8 ble_online, u8 offline, u32 offline_age_ms,
                                      char *command, u16 command_size)
{
    int length;
    if (snapshot == RT_NULL || !snapshot->telemetry_valid) return 0;
    /* 离线补传增加标记和相对缓存时长，云端可区分实时与历史数据。 */
    if (offline)
        length = snprintf(command, command_size,
            "AT+MQTTPUB=0,\"sensor_data\",\"{\\\"device_id\\\":\\\"202102\\\",\\\"Tin\\\":%.2f,\\\"Tout\\\":%.2f,\\\"LXin\\\":%.1f,\\\"battery_mv\\\":%u,\\\"status\\\":%u,\\\"ble_online\\\":%u,\\\"offline\\\":1,\\\"offline_age_ms\\\":%lu}\",0,0",
            snapshot->tin_c, snapshot->tout_c, snapshot->illuminance_lux,
            snapshot->battery_mv, snapshot->device_status, ble_online, offline_age_ms);
    else
        length = snprintf(command, command_size,
            "AT+MQTTPUB=0,\"sensor_data\",\"{\\\"device_id\\\":\\\"202102\\\",\\\"Tin\\\":%.2f,\\\"Tout\\\":%.2f,\\\"LXin\\\":%.1f,\\\"battery_mv\\\":%u,\\\"status\\\":%u,\\\"ble_online\\\":%u}\",0,0",
            snapshot->tin_c, snapshot->tout_c, snapshot->illuminance_lux,
            snapshot->battery_mv, snapshot->device_status, ble_online);
    /* snprintf 返回所需长度；等于或超过容量说明输出被截断，禁止发送。 */
    return length > 0 && length < command_size;
}

static void mqtt_outbox_thread_entry(void *parameter)
{
    mqtt_outbox_item_t item;
    (void)parameter;

    while (1)
    {
        if (!cloud_service_is_online())
        {
            /*
             * 断线时持续消费普通队列，避免它先被填满。只有 TELEMETRY 类型
             * 会进入历史队列；离线心跳没有补传价值，取出后自然丢弃。
             * 关键事件仍保留在关键消息队列，等待网络恢复。
             */
            if (rt_mq_recv(mqtt_normal_mq, &item, sizeof(item),
                           rt_tick_from_millisecond(100)) == RT_EOK)
                mqtt_offline_enqueue(&item);
            continue;
        }
        if (rt_mq_recv(mqtt_critical_mq, &item, sizeof(item), 0) != RT_EOK)
        {
            /* 调度顺序：关键消息 > 历史遥测 FIFO > 新普通消息。 */
            {
                mqtt_offline_telemetry_t offline_item;
                if (mqtt_offline_dequeue(&offline_item))
                {
                    if (!mqtt_make_telemetry_command(&offline_item.telemetry,
                                                     offline_item.ble_online, 1,
                                                     (u32)((rt_tick_get() - offline_item.captured_tick) *
                                                     (1000U / RT_TICK_PER_SECOND)),
                                                     item.command, sizeof(item.command))) continue;
                    item.length = (u16)strlen(item.command);
                    /*
                     * 当前以“AT 命令写入 ESP8266”为本地交付点；没有等待
                     * Broker 业务确认，因此发送后立即从历史队列移除。
                     */
                    uart3_write((const u8 *)item.command, item.length);
                    uart3_write((const u8 *)"\r\n", 2);
                    continue;
                }
            }
            if (rt_mq_recv(mqtt_normal_mq, &item, sizeof(item),
                           rt_tick_from_millisecond(20)) != RT_EOK) continue;
        }
        if (item.type == MQTT_OUTBOX_TELEMETRY)
        {
            /* 在线实时遥测在发送线程中延迟编码，生产者无需占用格式化时间。 */
            if (!mqtt_make_telemetry_command(&item.telemetry, item.ble_online, 0, 0,
                                             item.command, sizeof(item.command))) continue;
            item.length = (u16)strlen(item.command);
        }
        uart3_write((const u8 *)item.command, item.length);
        uart3_write((const u8 *)"\r\n", 2);
    }
}

void mqtt_outbox_init(void)
{
    /* 初始化函数具备幂等性，避免重复创建线程和消息队列。 */
    if (mqtt_thread != RT_NULL) return;
    mqtt_critical_drops = 0;
    mqtt_normal_drops = 0;
    mqtt_offline_head = 0;
    mqtt_offline_tail = 0;
    mqtt_offline_count = 0;
    mqtt_offline_drops = 0;
    /* 两个 RT-Thread 消息队列提供优先级隔离，静态循环队列负责断线历史。 */
    mqtt_critical_mq = rt_mq_create("mqcrit", sizeof(mqtt_outbox_item_t),
                                    MQTT_CRITICAL_QUEUE_DEPTH, RT_IPC_FLAG_PRIO);
    mqtt_normal_mq = rt_mq_create("mqnorm", sizeof(mqtt_outbox_item_t),
                                  MQTT_NORMAL_QUEUE_DEPTH, RT_IPC_FLAG_PRIO);
    if (mqtt_critical_mq == RT_NULL || mqtt_normal_mq == RT_NULL) return;
    mqtt_thread = rt_thread_create("mqtttx", mqtt_outbox_thread_entry, RT_NULL,
                                   MQTT_THREAD_STACK_SIZE, MQTT_THREAD_PRIORITY, 10);
    if (mqtt_thread != RT_NULL) rt_thread_startup(mqtt_thread);
}

u8 mqtt_outbox_submit(const char *command, mqtt_priority_t priority)
{
    mqtt_outbox_item_t item;
    rt_mq_t queue;
    u16 length;
    if (command == RT_NULL) return 0;
    length = (u16)strlen(command);
    if (length == 0 || length >= MQTT_OUTBOX_COMMAND_MAX) return 0;
    queue = priority == MQTT_PRIORITY_CRITICAL ? mqtt_critical_mq : mqtt_normal_mq;
    if (queue == RT_NULL) return 0;
    memset(&item, 0, sizeof(item));
    item.type = MQTT_OUTBOX_COMMAND;
    item.length = length;
    /* length 单独保存，发送时不依赖字符串终止符。 */
    memcpy(item.command, command, length);
    if (rt_mq_send(queue, &item, sizeof(item)) != RT_EOK)
    {
        if (priority == MQTT_PRIORITY_CRITICAL) ++mqtt_critical_drops;
        else ++mqtt_normal_drops;
        return 0;
    }
    return 1;
}

u8 mqtt_outbox_submit_telemetry(const gateway_state_snapshot_t *snapshot, u8 ble_online)
{
    mqtt_outbox_item_t item;
    if (snapshot == RT_NULL || !snapshot->telemetry_valid || mqtt_normal_mq == RT_NULL) return 0;
    memset(&item, 0, sizeof(item));
    item.type = MQTT_OUTBOX_TELEMETRY;
    /* 按值复制快照，调用者返回后消息仍然拥有独立、稳定的数据。 */
    item.telemetry = *snapshot;
    item.ble_online = ble_online ? 1U : 0U;
    if (rt_mq_send(mqtt_normal_mq, &item, sizeof(item)) != RT_EOK)
    {
        ++mqtt_normal_drops;
        return 0;
    }
    return 1;
}

u32 mqtt_outbox_drop_count(mqtt_priority_t priority)
{
    return priority == MQTT_PRIORITY_CRITICAL ? mqtt_critical_drops : mqtt_normal_drops;
}

u16 mqtt_outbox_offline_pending_count(void)
{
    u16 count;
    /* 与 enqueue/dequeue 同步，避免 LCD 读取到正在修改的计数。 */
    rt_enter_critical();
    count = mqtt_offline_count;
    rt_exit_critical();
    return count;
}

u32 mqtt_outbox_offline_drop_count(void)
{
    return mqtt_offline_drops;
}
