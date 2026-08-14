#ifndef __BLE_FRAME_H__
#define __BLE_FRAME_H__

/**
 * @file ble_frame.h
 * @brief 网关与下层 BLE 设备共用的二进制帧定义。
 *
 * 帧格式：AA 55 | version | type | sequence(小端) | length(小端) |
 *          payload | CRC16-Modbus(小端)。
 */
#include "sys.h"

#define BLE_FRAME_HEAD0        0xAA
#define BLE_FRAME_HEAD1        0x55
#define BLE_PROTOCOL_VERSION   0x01
#define BLE_MAX_PAYLOAD        128
#define BLE_FRAME_OVERHEAD     10 /* 帧头(2) + 固定头部(6) + CRC16(2) */
#define BLE_MAX_FRAME_SIZE     (BLE_FRAME_OVERHEAD + BLE_MAX_PAYLOAD)

typedef enum
{
    /* 下层设备主动上报。 */
    BLE_MSG_HEARTBEAT = 0x01,
    BLE_MSG_TELEMETRY = 0x10,
    BLE_MSG_ALARM     = 0x11,
    /* 网关主动下发。 */
    BLE_MSG_CONTROL   = 0x20,
    BLE_MSG_CONFIG    = 0x21,
    /* 对控制/配置请求的响应。 */
    BLE_MSG_ACK       = 0x7E,
    BLE_MSG_NACK      = 0x7F
} ble_message_type_t;

/* 网关和下层 BLE 设备共用的控制标识。 */
typedef enum
{
    BLE_CONTROL_MODE = 0x01,
    BLE_CONTROL_FAN_SPEED = 0x02,
    BLE_CONTROL_AC_SPEED = 0x03,
    BLE_CONTROL_LED_BRIGHTNESS = 0x04,
    BLE_CONTROL_TIN_HIGH = 0x05,
    BLE_CONTROL_TIN_LOW = 0x06,
    BLE_CONTROL_TEMPERATURE_DELTA = 0x07,
    BLE_CONTROL_LUX_THRESHOLD = 0x08,
    BLE_CONTROL_TIME = 0x09,
    BLE_CONTROL_TIME_BEGIN = 0x0A,
    BLE_CONTROL_TIME_END = 0x0B
} ble_control_id_t;

typedef struct
{
    u8 version;
    u8 type;
    u16 sequence;
    u16 length;
    u8 payload[BLE_MAX_PAYLOAD];
} ble_frame_t;

/* 逐字节状态机允许解析器从连续 UART 字节流中恢复帧边界。 */
typedef enum
{
    BLE_PARSER_WAIT_HEAD0,
    BLE_PARSER_WAIT_HEAD1,
    BLE_PARSER_VERSION,
    BLE_PARSER_TYPE,
    BLE_PARSER_SEQUENCE_L,
    BLE_PARSER_SEQUENCE_H,
    BLE_PARSER_LENGTH_L,
    BLE_PARSER_LENGTH_H,
    BLE_PARSER_PAYLOAD,
    BLE_PARSER_CRC_L,
    BLE_PARSER_CRC_H
} ble_parser_state_t;

typedef struct
{
    ble_parser_state_t state;
    ble_frame_t frame;
    u16 payload_index;
    u16 received_crc;
} ble_frame_parser_t;

void ble_frame_parser_init(ble_frame_parser_t *parser);
/** 喂入一个字节；返回 1 表示输出了一帧 CRC 正确的完整消息。 */
u8 ble_frame_parser_feed(ble_frame_parser_t *parser, u8 byte, ble_frame_t *frame);
/** 计算 CRC16-Modbus，初值 0xFFFF，多项式 0xA001。 */
u16 ble_crc16_modbus(const u8 *data, u16 length);
/** 编码成功返回总帧长；参数或输出容量不合法时返回 0。 */
u16 ble_frame_encode(const ble_frame_t *frame, u8 *output, u16 output_size);

#endif
