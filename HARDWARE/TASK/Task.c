/**
 * @file Task.c
 * @brief 网关业务编排层。
 *
 * 本模块组合 BLE、云端和 MQTT 服务，负责周期上报、LCD 显示、本地告警
 * 以及控制结果回传。UART 字节接收与协议解析由底层服务完成。
 */
#include "Task.h"
#include "lcd.h"
#include "beep.h"
#include "led.h"
#include "ble_service.h"
#include "cloud_service.h"
#include "gateway_state.h"
#include "mqtt_outbox.h"
#include "wifista.h"
#include <rtthread.h>
#include <stdio.h>

#define DEVICE_ID "202102" /* 云端识别本台网关的固定设备编号。 */

/* 周期遥测和网关心跳由 RT-Thread 定时器触发。 */
static rt_timer_t telemetry_timer;
static rt_timer_t heartbeat_timer;

/** 将二进制控制 ID 转换为云端可读的命令名称。 */
static const char *command_name(ble_control_id_t id)
{
    switch (id)
    {
    case BLE_CONTROL_MODE: return "mode";
    case BLE_CONTROL_FAN_SPEED: return "fan_speed";
    case BLE_CONTROL_AC_SPEED: return "ac_speed";
    case BLE_CONTROL_LED_BRIGHTNESS: return "led_brightness";
    case BLE_CONTROL_TIN_HIGH: return "tin_high";
    case BLE_CONTROL_TIN_LOW: return "tin_low";
    case BLE_CONTROL_TEMPERATURE_DELTA: return "temperature_delta";
    case BLE_CONTROL_LUX_THRESHOLD: return "lux_threshold";
    default: return "unknown";
    }
}

static void command_result_callback(const ble_command_result_t *result)
{
    char command[MQTT_OUTBOX_COMMAND_MAX];
    int length;
    if (result == RT_NULL) return;

    /*
     * 回调运行在 BLE 控制线程中。这里只生成 MQTT 命令并放入关键队列，
     * 实际 USART3 发送交给 mqtttx 线程，避免阻塞 BLE 控制闭环。
     */
    length = snprintf(command, sizeof(command),
        "AT+MQTTPUB=0,\"command_result\",\"{\\\"device_id\\\":\\\"%s\\\",\\\"sequence\\\":%u,\\\"command\\\":\\\"%s\\\",\\\"result\\\":%u,\\\"attempts\\\":%u}\",0,0",
        DEVICE_ID, result->sequence, command_name(result->control_id),
        result->result, result->attempts);
    if (length > 0 && length < (int)sizeof(command))
        mqtt_outbox_submit(command, MQTT_PRIORITY_CRITICAL);
}

static void telemetry_timer_callback(void *parameter)
{
    gateway_state_snapshot_t snapshot;
    (void)parameter;
    /* 读取一致性快照，避免在多字段更新中途获得混合数据。 */
    gateway_state_snapshot(&snapshot);
    if (!snapshot.telemetry_valid) return;

    /*
     * 提交结构化快照而不是 MQTT 字符串：在线时再编码；离线时只缓存紧凑
     * 遥测结构，从而降低 60 条断线缓存的静态 RAM 占用。
     */
    mqtt_outbox_submit_telemetry(&snapshot, ble_service_is_online());
}

/** 每 10 秒发布一次网关心跳；历史心跳没有补传价值，因此不进入离线缓存。 */
static void heartbeat_timer_callback(void *parameter)
{
    char command[MQTT_OUTBOX_COMMAND_MAX];
    int length;
    (void)parameter;
    length = snprintf(command, sizeof(command),
        "AT+MQTTPUB=0,\"heartbeat\",\"{\\\"device_id\\\":\\\"%s\\\",\\\"ble_online\\\":%u}\",0,0",
        DEVICE_ID, ble_service_is_online());
    if (length > 0 && length < (int)sizeof(command))
        mqtt_outbox_submit(command, MQTT_PRIORITY_NORMAL);
}

static void display_thread_entry(void *parameter)
{
    gateway_state_snapshot_t snapshot;
    char line[40];
    (void)parameter;
    LCD_Clear(WHITE);
    while (1)
    {
        /* 显示线程仅访问状态接口，不直接操作 BLE/ESP8266 通信链路。 */
        gateway_state_snapshot(&snapshot);
        snprintf(line, sizeof(line), "BLE:%s CLOUD:%s", ble_service_is_online() ? "ON" : "OFF",
                 cloud_service_is_online() ? "ON" : "OFF");
        LCD_ShowString(10, 10, 300, 20, 16, (u8 *)line);
        snprintf(line, sizeof(line), "Tin:%.2f Tout:%.2f", snapshot.tin_c, snapshot.tout_c);
        LCD_ShowString(10, 40, 300, 20, 16, (u8 *)line);
        snprintf(line, sizeof(line), "Lux:%.1f Bat:%umV", snapshot.illuminance_lux, snapshot.battery_mv);
        LCD_ShowString(10, 70, 300, 20, 16, (u8 *)line);
        snprintf(line, sizeof(line), "Alarm:%u Status:%u", snapshot.alarm_code, snapshot.device_status);
        LCD_ShowString(10, 100, 300, 20, 16, (u8 *)line);
        /* Cache 表示待补传条数，Drop 表示离线缓存满后丢弃的新遥测数。 */
        snprintf(line, sizeof(line), "Cache:%u Drop:%lu", mqtt_outbox_offline_pending_count(),
                 mqtt_outbox_offline_drop_count());
        LCD_ShowString(10, 130, 300, 20, 16, (u8 *)line);
        rt_thread_mdelay(500);
    }
}

static void alarm_thread_entry(void *parameter)
{
    gateway_state_snapshot_t snapshot;
    u8 last_alarm = 0xFF;
    u8 link_loss_reported = 0;
    char command[MQTT_OUTBOX_COMMAND_MAX];
    int length;
    (void)parameter;
    while (1)
    {
        gateway_state_snapshot(&snapshot);
        /* 本开发板的蜂鸣器和 LED 均为低电平有效。 */
        BEEP = snapshot.alarm_code ? 0 : 1;
        LED0 = snapshot.alarm_code ? 0 : 1;

        /* 告警码只在发生变化时处理，避免 100 ms 周期重复发布同一故障。 */
        if (snapshot.alarm_code != last_alarm)
        {
            last_alarm = snapshot.alarm_code;
            if (snapshot.alarm_code)
            {
                length = snprintf(command, sizeof(command),
                    "AT+MQTTPUB=0,\"fault\",\"{\\\"device_id\\\":\\\"%s\\\",\\\"alarm_code\\\":%u}\",0,0",
                    DEVICE_ID, snapshot.alarm_code);
                if (length > 0 && length < (int)sizeof(command))
                    mqtt_outbox_submit(command, MQTT_PRIORITY_CRITICAL);
            }
        }
        /* 一个 BLE 失联周期只上报一次；链路恢复后重新允许下一次上报。 */
        if (ble_service_poll_link_lost() && !link_loss_reported)
        {
            link_loss_reported = 1;
            length = snprintf(command, sizeof(command),
                "AT+MQTTPUB=0,\"fault\",\"{\\\"device_id\\\":\\\"%s\\\",\\\"alarm\\\":\\\"ble_link_lost\\\"}\",0,0", DEVICE_ID);
            if (length > 0 && length < (int)sizeof(command))
                mqtt_outbox_submit(command, MQTT_PRIORITY_CRITICAL);
        }
        if (ble_service_is_online()) link_loss_reported = 0;
        rt_thread_mdelay(100);
    }
}

void TaskInit(void)
{
    rt_thread_t display_thread;
    rt_thread_t alarm_thread;

    /* 先建立共享状态和发送队列，再启动可能产生回调的 BLE 服务。 */
    gateway_state_init();
    mqtt_outbox_init();
    ble_service_init();
    ble_service_set_command_result_callback(command_result_callback);

    /* 启动阶段同步尝试配网；失败后 cloudrx 线程仍会在后台继续重连。 */
    atk_8266_wifista_config();
    cloud_service_init();

    /* 告警优先级 10 高于显示优先级 12，保证本地故障响应及时。 */
    display_thread = rt_thread_create("display", display_thread_entry, RT_NULL, 1024, 12, 10);
    if (display_thread != RT_NULL) rt_thread_startup(display_thread);
    alarm_thread = rt_thread_create("alarm", alarm_thread_entry, RT_NULL, 768, 10, 10);
    if (alarm_thread != RT_NULL) rt_thread_startup(alarm_thread);

    /* 遥测周期 2 秒，网关心跳周期 10 秒。 */
    telemetry_timer = rt_timer_create("telemetry", telemetry_timer_callback, RT_NULL,
                                      rt_tick_from_millisecond(2000), RT_TIMER_FLAG_PERIODIC);
    heartbeat_timer = rt_timer_create("heartbeat", heartbeat_timer_callback, RT_NULL,
                                      rt_tick_from_millisecond(10000), RT_TIMER_FLAG_PERIODIC);
    if (telemetry_timer != RT_NULL) rt_timer_start(telemetry_timer);
    if (heartbeat_timer != RT_NULL) rt_timer_start(heartbeat_timer);
}
