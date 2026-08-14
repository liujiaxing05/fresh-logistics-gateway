/**
 * @file ble_service.c
 * @brief BLE 接收解析、在线状态管理及可靠控制下发。
 *
 * USART2 ISR 只缓存字节并释放信号量；blerx 线程完成组帧，blectl 线程
 * 负责控制命令编码、ACK 等待和超时重传。
 */
#include "ble_service.h"
#include "ble_frame.h"
#include "usart2.h"
#include "gateway_state.h"
#include <rthw.h>
#include <rtthread.h>
#include <string.h>

#define BLE_RX_THREAD_STACK_SIZE 768
#define BLE_RX_THREAD_PRIORITY   5
#define BLE_COMMAND_THREAD_STACK_SIZE 768
#define BLE_COMMAND_THREAD_PRIORITY   6
#define BLE_ONLINE_TIMEOUT_MS    15000
#define BLE_COMMAND_QUEUE_DEPTH  8
#define BLE_COMMAND_ACK_TIMEOUT_MS 500
#define BLE_COMMAND_MAX_ATTEMPTS 3

static struct rt_semaphore ble_rx_sem;
static rt_thread_t ble_rx_thread;
static ble_frame_parser_t ble_parser;
static volatile rt_tick_t ble_last_rx_tick;
static volatile u8 ble_link_seen;
static volatile u8 ble_link_lost_reported;
static volatile u32 ble_valid_frame_count;
static u16 ble_next_sequence;
static rt_mq_t ble_command_mq;
static struct rt_semaphore ble_ack_sem;
static rt_thread_t ble_command_thread;
static volatile u16 ble_waiting_sequence;
static volatile u8 ble_ack_received;
static volatile ble_command_result_code_t ble_ack_result;
static ble_command_result_callback_t ble_result_callback;

/* 消息队列元素采用定长结构，避免在线程间传递动态内存所有权。 */
typedef struct
{
    u16 sequence;
    ble_control_id_t control_id;
    u16 length;
    u8 data[BLE_MAX_PAYLOAD - 1];
} ble_command_request_t;

static s16 read_i16_le(const u8 *data)
{
    /* 先组合无符号值再转换，保证负温度的二进制补码保持不变。 */
    return (s16)((u16)data[0] | ((u16)data[1] << 8));
}

static u32 read_u32_le(const u8 *data)
{
    return (u32)data[0] | ((u32)data[1] << 8) |
           ((u32)data[2] << 16) | ((u32)data[3] << 24);
}

static void ble_report_command_result(const ble_command_request_t *request,
                                      ble_command_result_code_t result,
                                      u8 attempts)
{
    ble_command_result_t command_result;
    if (ble_result_callback == RT_NULL) return;
    /* 在调用者线程上下文同步执行；回调内不得长时间阻塞。 */
    command_result.sequence = request->sequence;
    command_result.control_id = request->control_id;
    command_result.result = result;
    command_result.attempts = attempts;
    ble_result_callback(&command_result);
}

static void ble_handle_command_response(const ble_frame_t *frame)
{
    ble_command_result_code_t result;

    /* 只有类型、原请求类型和 sequence 全部匹配才可唤醒等待者。 */
    if ((frame->type != BLE_MSG_ACK && frame->type != BLE_MSG_NACK) ||
        frame->length < 2 || frame->payload[0] != BLE_MSG_CONTROL ||
        frame->sequence != ble_waiting_sequence)
        return;

    result = (frame->type == BLE_MSG_ACK && frame->payload[1] == 0) ?
             BLE_COMMAND_RESULT_OK : BLE_COMMAND_RESULT_REJECTED;
    ble_ack_result = result;
    ble_ack_received = 1;
    rt_sem_release(&ble_ack_sem);
}

static void ble_dispatch_frame(const ble_frame_t *frame)
{
    /* 任意合法协议帧都可证明链路当前可用。 */
    ble_last_rx_tick = rt_tick_get();
    ble_link_seen = 1;
    ble_link_lost_reported = 0;
    ++ble_valid_frame_count;
    if (frame->type == BLE_MSG_TELEMETRY && frame->length >= 11)
    {
        /* 遥测负载：Tin(2)+Tout(2)+Lux(4)+Battery(2)+Status(1)。 */
        gateway_state_update_telemetry(read_i16_le(&frame->payload[0]),
                                       read_i16_le(&frame->payload[2]),
                                       read_u32_le(&frame->payload[4]),
                                       (u16)frame->payload[8] | ((u16)frame->payload[9] << 8),
                                       frame->payload[10]);
    }
    else if (frame->type == BLE_MSG_ALARM && frame->length >= 1)
    {
        gateway_state_update_alarm(frame->payload[0]);
    }
    else if (frame->type == BLE_MSG_ACK || frame->type == BLE_MSG_NACK)
    {
        ble_handle_command_response(frame);
    }
}

static void ble_rx_thread_entry(void *parameter)
{
    u8 byte;
    ble_frame_t frame;
    (void)parameter;
    while (1)
    {
        /* 信号量只表示“可能有数据”；醒来后必须一次排空环形缓冲。 */
        rt_sem_take(&ble_rx_sem, RT_WAITING_FOREVER);
        while (ble_uart_rx_pop(&byte))
        {
            if (ble_frame_parser_feed(&ble_parser, byte, &frame))
                ble_dispatch_frame(&frame);
        }
    }
}

static void ble_command_thread_entry(void *parameter)
{
    ble_command_request_t request;
    ble_frame_t frame;
    u8 encoded[BLE_MAX_FRAME_SIZE];
    u16 encoded_length;
    u8 attempts;
    (void)parameter;

    while (1)
    {
        /* 单线程串行处理控制请求，确保任一时刻只等待一个 sequence 的 ACK。 */
        if (rt_mq_recv(ble_command_mq, &request, sizeof(request), RT_WAITING_FOREVER) != RT_EOK)
            continue;

        frame.version = BLE_PROTOCOL_VERSION;
        frame.type = BLE_MSG_CONTROL;
        frame.sequence = request.sequence;
        frame.length = request.length + 1;
        frame.payload[0] = (u8)request.control_id;
        if (request.length) memcpy(&frame.payload[1], request.data, request.length);
        encoded_length = ble_frame_encode(&frame, encoded, sizeof(encoded));
        if (encoded_length == 0)
        {
            ble_report_command_result(&request, BLE_COMMAND_RESULT_REJECTED, 0);
            continue;
        }

        /* 每次发送前清除历史信号量，避免上一请求的迟到 ACK 误触发。 */
        for (attempts = 1; attempts <= BLE_COMMAND_MAX_ATTEMPTS; ++attempts)
        {
            while (rt_sem_take(&ble_ack_sem, 0) == RT_EOK);
            ble_waiting_sequence = request.sequence;
            ble_ack_received = 0;
            ble_uart_write(encoded, encoded_length);

            if (rt_sem_take(&ble_ack_sem,
                            rt_tick_from_millisecond(BLE_COMMAND_ACK_TIMEOUT_MS)) == RT_EOK &&
                ble_ack_received)
            {
                /* ACK/NACK 已由接收线程校验 sequence，并写入 ble_ack_result。 */
                ble_waiting_sequence = 0;
                ble_report_command_result(&request, ble_ack_result, attempts);
                break;
            }
        }

        if (attempts > BLE_COMMAND_MAX_ATTEMPTS)
        {
            ble_waiting_sequence = 0;
            ble_report_command_result(&request, BLE_COMMAND_RESULT_TIMEOUT,
                                      BLE_COMMAND_MAX_ATTEMPTS);
        }
    }
}

void ble_service_init(void)
{
    /* 初始化顺序：状态 -> IPC 对象 -> 工作线程。 */
    ble_frame_parser_init(&ble_parser);
    ble_last_rx_tick = 0;
    ble_link_seen = 0;
    ble_link_lost_reported = 0;
    ble_valid_frame_count = 0;
    ble_next_sequence = 1;
    ble_waiting_sequence = 0;
    ble_result_callback = RT_NULL;
    rt_sem_init(&ble_rx_sem, "blerx", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&ble_ack_sem, "bleack", 0, RT_IPC_FLAG_PRIO);
    ble_command_mq = rt_mq_create("blectl", sizeof(ble_command_request_t),
                                  BLE_COMMAND_QUEUE_DEPTH, RT_IPC_FLAG_PRIO);
    ble_rx_thread = rt_thread_create("blerx", ble_rx_thread_entry, RT_NULL,
                                     BLE_RX_THREAD_STACK_SIZE,
                                     BLE_RX_THREAD_PRIORITY, 10);
    if (ble_rx_thread != RT_NULL) rt_thread_startup(ble_rx_thread);
    ble_command_thread = rt_thread_create("blectl", ble_command_thread_entry, RT_NULL,
                                          BLE_COMMAND_THREAD_STACK_SIZE,
                                          BLE_COMMAND_THREAD_PRIORITY, 10);
    if (ble_command_thread != RT_NULL) rt_thread_startup(ble_command_thread);
}

void ble_service_set_command_result_callback(ble_command_result_callback_t callback)
{
    ble_result_callback = callback;
}

void ble_service_notify_rx_isr(void)
{
    /* RT-Thread 信号量可在中断上下文释放；实际解析延后到线程。 */
    rt_sem_release(&ble_rx_sem);
}

u8 ble_service_is_online(void)
{
    /* 从未收到合法帧时 last_rx_tick 为 0，明确返回离线。 */
    if (ble_last_rx_tick == 0) return 0;
    return (rt_tick_get() - ble_last_rx_tick) < rt_tick_from_millisecond(BLE_ONLINE_TIMEOUT_MS);
}

u8 ble_service_poll_link_lost(void)
{
    /* 只有“曾经在线但现在超时”才构成失联事件，冷启动不报警。 */
    if (!ble_link_seen || ble_service_is_online() || ble_link_lost_reported) return 0;
    ble_link_lost_reported = 1;
    return 1;
}

u16 ble_service_send_control(ble_control_id_t id, const u8 *data, u16 length)
{
    ble_command_request_t request;
    rt_base_t level;
    if (ble_command_mq == RT_NULL || length > BLE_MAX_PAYLOAD - 1) return 0;

    /* sequence 在关中断临界区内分配，允许多个业务上下文并发提交。 */
    level = rt_hw_interrupt_disable();
    request.sequence = ble_next_sequence++;
    rt_hw_interrupt_enable(level);
    request.control_id = id;
    request.length = length;
    if (length) memcpy(request.data, data, length);
    if (!ble_service_is_online())
    {
        /* 离线时不占用命令队列，立即通过回调反馈失败。 */
        ble_report_command_result(&request, BLE_COMMAND_RESULT_OFFLINE, 0);
        return 0;
    }
    if (rt_mq_send(ble_command_mq, &request, sizeof(request)) != RT_EOK)
    {
        /* 队列满表示下发速度超过 BLE 控制线程的处理能力。 */
        ble_report_command_result(&request, BLE_COMMAND_RESULT_QUEUE_FULL, 0);
        return 0;
    }
    return request.sequence;
}

u16 ble_service_send_control_i32(ble_control_id_t id, s32 value)
{
    u8 data[4];
    data[0] = (u8)value;
    data[1] = (u8)(value >> 8);
    data[2] = (u8)(value >> 16);
    data[3] = (u8)(value >> 24);
    return ble_service_send_control(id, data, sizeof(data));
}
