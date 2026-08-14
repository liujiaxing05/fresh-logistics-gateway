#ifndef __MQTT_OUTBOX_H__
#define __MQTT_OUTBOX_H__

/**
 * @file mqtt_outbox.h
 * @brief MQTT 异步发送、优先级调度和断线遥测缓存接口。
 */
#include "sys.h"
#include "gateway_state.h"

#define MQTT_OUTBOX_COMMAND_MAX 384          /* 单条 ESP8266 AT 发布命令最大长度。 */
#define MQTT_OFFLINE_TELEMETRY_CAPACITY 60U /* 2 秒采样时约覆盖 120 秒断网。 */

typedef enum
{
    MQTT_PRIORITY_CRITICAL = 0, /* 告警和控制结果。 */
    MQTT_PRIORITY_NORMAL = 1    /* 遥测和心跳。 */
} mqtt_priority_t;

void mqtt_outbox_init(void);
/** 提交已经拼接完成的 AT+MQTTPUB 命令。 */
u8 mqtt_outbox_submit(const char *command, mqtt_priority_t priority);
/** 提交结构化遥测；离线时该类型可转存到静态循环队列。 */
u8 mqtt_outbox_submit_telemetry(const gateway_state_snapshot_t *snapshot, u8 ble_online);
/** 获取 RT-Thread 消息队列满导致的累计丢弃数。 */
u32 mqtt_outbox_drop_count(mqtt_priority_t priority);
/** 获取当前等待补传的断线遥测数量。 */
u16 mqtt_outbox_offline_pending_count(void);
/** 获取静态断线队列满后累计丢弃的新遥测数量。 */
u32 mqtt_outbox_offline_drop_count(void);

#endif
