/**
 * @file cloud_service.c
 * @brief ESP8266 串口行解析、云端控制路由与自动重连。
 *
 * cloudrx 线程是 USART3 接收环形缓冲的唯一常规消费者。在线时解析模块
 * 状态和订阅消息；离线时由同一线程执行 AT 重连，避免两个消费者竞争响应。
 */
#include "cloud_service.h"
#include "usart3.h"
#include "ble_service.h"
#include "wifista.h"
#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CLOUD_RX_LINE_MAX 512U
#define CLOUD_RECONNECT_INTERVAL_MS 5000U

static struct rt_semaphore cloud_rx_sem;
static volatile u8 cloud_online;
static volatile u8 cloud_initialized;

/**
 * 在受控 JSON 文本中定位字段值。
 * @note 这是轻量字符串解析，不是完整 JSON 解析器；输入格式必须稳定。
 */
static const char *cloud_value(const char *line, const char *key)
{
    static char quoted_key[32]; /* 仅 cloudrx 单线程调用，静态缓冲不会并发冲突。 */
    const char *p;
    snprintf(quoted_key, sizeof(quoted_key), "\"%s\"", key);
    p = strstr(line, quoted_key);
    if (p == RT_NULL) return RT_NULL;
    p += strlen(quoted_key);
    while (*p == ' ' || *p == '\t') ++p;
    if (*p != ':') return RT_NULL;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\"') ++p;
    return p;
}

static u8 cloud_int(const char *line, const char *key, s32 *value)
{
    char *end;
    const char *p = cloud_value(line, key);
    if (p == RT_NULL) return 0;
    /* end == p 表示没有成功解析任何数字。 */
    *value = (s32)strtol(p, &end, 10);
    return end != p;
}

static u8 cloud_float_x100(const char *line, const char *key, s32 *value)
{
    char *end;
    const char *p = cloud_value(line, key);
    float number;
    if (p == RT_NULL) return 0;
    number = strtof(p, &end);
    if (end == p) return 0;
    /* BLE 阈值协议使用放大 100 倍的定点整数，避免传输浮点二进制。 */
    *value = (s32)(number * 100.0f);
    return 1;
}

static void cloud_route_i32(const char *line, const char *key,
                            ble_control_id_t id, s32 min, s32 max)
{
    s32 value;
    /* 越界字段直接忽略，不向下层发送可能危险的执行器参数。 */
    if (cloud_int(line, key, &value) && value >= min && value <= max)
        ble_service_send_control_i32(id, value);
}

static void cloud_route_commands(const char *line)
{
    const char *mode = cloud_value(line, "mode");
    s32 value;
    if (mode != RT_NULL)
    {
        if (strncmp(mode, "open", 4) == 0) ble_service_send_control_i32(BLE_CONTROL_MODE, 1);
        else if (strncmp(mode, "close", 5) == 0) ble_service_send_control_i32(BLE_CONTROL_MODE, 0);
    }
    /* 一条云端消息可携带多个字段，每个字段形成独立 BLE 控制请求。 */
    cloud_route_i32(line, "SpeedM1", BLE_CONTROL_FAN_SPEED, 0, 100);
    cloud_route_i32(line, "SpeedM2", BLE_CONTROL_AC_SPEED, 0, 100);
    cloud_route_i32(line, "SpeedM3", BLE_CONTROL_LED_BRIGHTNESS, 0, 100);
    if (cloud_float_x100(line, "TinDH", &value)) ble_service_send_control_i32(BLE_CONTROL_TIN_HIGH, value);
    if (cloud_float_x100(line, "TinDL", &value)) ble_service_send_control_i32(BLE_CONTROL_TIN_LOW, value);
    if (cloud_float_x100(line, "TG", &value)) ble_service_send_control_i32(BLE_CONTROL_TEMPERATURE_DELTA, value);
    cloud_route_i32(line, "LXD", BLE_CONTROL_LUX_THRESHOLD, 0, 200000);
}

static void cloud_process_line(char *line)
{
    /* ESP8266 主动状态文本驱动 cloud_online；当前没有额外的主动探测。 */
    if (strstr(line, "MQTTCONNECTED") != RT_NULL || strstr(line, "MQTT CONNECTED") != RT_NULL)
        cloud_online = 1;
    if (strstr(line, "MQTTDISCONNECTED") != RT_NULL || strstr(line, "WIFI DISCONNECT") != RT_NULL)
        cloud_online = 0;
    /* 只有包含 JSON 起始符的行才进入业务命令路由。 */
    if (strchr(line, '{') != RT_NULL) cloud_route_commands(line);
}

static void cloud_rx_thread_entry(void *parameter)
{
    char line[CLOUD_RX_LINE_MAX];
    u16 length = 0;
    u8 byte;
    rt_tick_t last_reconnect_tick = 0;
    (void)parameter;
    while (1)
    {
        /*
         * 离线后每 5 秒尝试一轮完整配网。重连函数内部最多重试 3 次，
         * 返回后重新记录时间，防止失败时立即无间隔地复位模块。
         */
        if (!cloud_online &&
            (last_reconnect_tick == 0 ||
             rt_tick_get() - last_reconnect_tick >=
             rt_tick_from_millisecond(CLOUD_RECONNECT_INTERVAL_MS)))
        {
            /* 此线程独占 UART3 接收缓冲，重连过程不会与命令解析竞争 AT 响应。 */
            atk_8266_wifista_config();
            last_reconnect_tick = rt_tick_get();
            continue;
        }
        /* 500 ms 超时使线程即使没有串口数据也能检查离线重连条件。 */
        rt_sem_take(&cloud_rx_sem, rt_tick_from_millisecond(500));
        while (uart3_rx_pop(&byte))
        {
            /* ESP8266 AT 输出按 CRLF 分行；忽略 CR，以 LF 结束一行。 */
            if (byte == '\r') continue;
            if (byte == '\n')
            {
                if (length) { line[length] = '\0'; cloud_process_line(line); length = 0; }
            }
            else if (length < sizeof(line) - 1U) line[length++] = (char)byte;
            else length = 0;
        }
    }
}

void cloud_service_init(void)
{
    rt_thread_t thread;
    /* 防止重复创建同名信号量和接收线程。 */
    if (cloud_initialized) return;
    cloud_online = 0;
    rt_sem_init(&cloud_rx_sem, "cloudrx", 0, RT_IPC_FLAG_PRIO);
    thread = rt_thread_create("cloudrx", cloud_rx_thread_entry, RT_NULL, 1024, 7, 10);
    if (thread != RT_NULL) rt_thread_startup(thread);
    cloud_initialized = 1;
}

void cloud_service_notify_rx_isr(void)
{
    /* 初始化前收到的字节由启动阶段 atk_8266_send_cmd() 主动轮询。 */
    if (cloud_initialized) rt_sem_release(&cloud_rx_sem);
}

u8 cloud_service_is_online(void)
{
    return cloud_online;
}

void cloud_service_set_online(u8 online)
{
    cloud_online = online ? 1 : 0;
}
