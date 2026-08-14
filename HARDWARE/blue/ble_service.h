#ifndef __BLE_SERVICE_H__
#define __BLE_SERVICE_H__

/**
 * @file ble_service.h
 * @brief BLE 链路服务对业务层公开的接口。
 */
#include "sys.h"
#include "ble_frame.h"

typedef enum
{
    /* result 字段同时覆盖下层应答和网关本地失败。 */
    BLE_COMMAND_RESULT_OK = 0,
    BLE_COMMAND_RESULT_REJECTED = 1,
    BLE_COMMAND_RESULT_TIMEOUT = 2,
    BLE_COMMAND_RESULT_QUEUE_FULL = 3,
    BLE_COMMAND_RESULT_OFFLINE = 4
} ble_command_result_code_t;

typedef struct
{
    u16 sequence;                 /* 本次控制请求的唯一序列号。 */
    ble_control_id_t control_id;
    ble_command_result_code_t result;
    u8 attempts;                  /* 实际发送次数；本地拒绝时为 0。 */
} ble_command_result_t;

typedef void (*ble_command_result_callback_t)(const ble_command_result_t *result);

void ble_service_init(void);
/** 由 USART2 接收中断调用，仅用于唤醒 BLE 接收线程。 */
void ble_service_notify_rx_isr(void);
/** 最近 15 秒内收到过合法帧则返回 1。 */
u8 ble_service_is_online(void);
/** 每次失联周期只返回一次 1，供告警线程生成边沿事件。 */
u8 ble_service_poll_link_lost(void);
void ble_service_set_command_result_callback(ble_command_result_callback_t callback);
u16 ble_service_send_control(ble_control_id_t id, const u8 *data, u16 length);
/** 将 32 位控制值按小端序编码后发送。 */
u16 ble_service_send_control_i32(ble_control_id_t id, s32 value);

#endif
