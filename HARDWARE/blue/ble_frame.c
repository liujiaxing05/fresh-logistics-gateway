/**
 * @file ble_frame.c
 * @brief BLE 二进制协议的编码、CRC 校验和流式解析状态机。
 */
#include "ble_frame.h"
#include <string.h>

static void ble_frame_parser_reset(ble_frame_parser_t *parser)
{
    /* 下一次输入从寻找第一个帧头字节开始。 */
    parser->state = BLE_PARSER_WAIT_HEAD0;
    parser->payload_index = 0;
    parser->received_crc = 0;
}

u16 ble_crc16_modbus(const u8 *data, u16 length)
{
    u16 crc = 0xFFFF;
    u16 i;

    /* 按最低位优先处理每个字节，与 Modbus CRC 定义一致。 */
    while (length--)
    {
        crc ^= *data++;
        for (i = 0; i < 8; ++i)
        {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
        }
    }
    return crc;
}

void ble_frame_parser_init(ble_frame_parser_t *parser)
{
    /* 清零整个上下文，便于首次使用和调试时观察确定状态。 */
    memset(parser, 0, sizeof(*parser));
    ble_frame_parser_reset(parser);
}

u8 ble_frame_parser_feed(ble_frame_parser_t *parser, u8 byte, ble_frame_t *frame)
{
    u8 crc_data[6 + BLE_MAX_PAYLOAD];
    u16 expected_crc;

    /* 每次只消费一个字节，调用方可直接从 UART 环形缓冲持续喂入。 */
    switch (parser->state)
    {
    case BLE_PARSER_WAIT_HEAD0:
        if (byte == BLE_FRAME_HEAD0) parser->state = BLE_PARSER_WAIT_HEAD1;
        break;
    case BLE_PARSER_WAIT_HEAD1:
        /* 第二个帧头不匹配则重新同步，后续合法帧仍可被识别。 */
        parser->state = (byte == BLE_FRAME_HEAD1) ? BLE_PARSER_VERSION : BLE_PARSER_WAIT_HEAD0;
        break;
    case BLE_PARSER_VERSION:
        parser->frame.version = byte;
        parser->state = BLE_PARSER_TYPE;
        break;
    case BLE_PARSER_TYPE:
        parser->frame.type = byte;
        parser->state = BLE_PARSER_SEQUENCE_L;
        break;
    case BLE_PARSER_SEQUENCE_L:
        parser->frame.sequence = byte;
        parser->state = BLE_PARSER_SEQUENCE_H;
        break;
    case BLE_PARSER_SEQUENCE_H:
        parser->frame.sequence |= ((u16)byte << 8);
        parser->state = BLE_PARSER_LENGTH_L;
        break;
    case BLE_PARSER_LENGTH_L:
        parser->frame.length = byte;
        parser->state = BLE_PARSER_LENGTH_H;
        break;
    case BLE_PARSER_LENGTH_H:
        parser->frame.length |= ((u16)byte << 8);
        if (parser->frame.version != BLE_PROTOCOL_VERSION || parser->frame.length > BLE_MAX_PAYLOAD)
        {
            /* 长度越界时立即丢帧，避免写出 payload 数组边界。 */
            ble_frame_parser_reset(parser);
        }
        else if (parser->frame.length == 0)
        {
            parser->state = BLE_PARSER_CRC_L;
        }
        else
        {
            parser->payload_index = 0;
            parser->state = BLE_PARSER_PAYLOAD;
        }
        break;
    case BLE_PARSER_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        if (parser->payload_index == parser->frame.length) parser->state = BLE_PARSER_CRC_L;
        break;
    case BLE_PARSER_CRC_L:
        parser->received_crc = byte;
        parser->state = BLE_PARSER_CRC_H;
        break;
    case BLE_PARSER_CRC_H:
        parser->received_crc |= ((u16)byte << 8);
        crc_data[0] = parser->frame.version;
        crc_data[1] = parser->frame.type;
        crc_data[2] = (u8)parser->frame.sequence;
        crc_data[3] = (u8)(parser->frame.sequence >> 8);
        crc_data[4] = (u8)parser->frame.length;
        crc_data[5] = (u8)(parser->frame.length >> 8);
        if (parser->frame.length) memcpy(&crc_data[6], parser->frame.payload, parser->frame.length);
        /* CRC 覆盖 version 到 payload，不包含 0xAA55 帧头和 CRC 自身。 */
        expected_crc = ble_crc16_modbus(crc_data, 6 + parser->frame.length);
        if (expected_crc == parser->received_crc)
        {
            *frame = parser->frame;
            ble_frame_parser_reset(parser);
            return 1;
        }
        ble_frame_parser_reset(parser);
        break;
    default:
        ble_frame_parser_reset(parser);
        break;
    }
    return 0;
}

u16 ble_frame_encode(const ble_frame_t *frame, u8 *output, u16 output_size)
{
    u16 crc;
    u16 total = BLE_FRAME_OVERHEAD + frame->length;

    /* 编码前统一验证协议版本、载荷上限和调用方缓冲区容量。 */
    if (frame->version != BLE_PROTOCOL_VERSION || frame->length > BLE_MAX_PAYLOAD || output_size < total)
        return 0;

    output[0] = BLE_FRAME_HEAD0;
    output[1] = BLE_FRAME_HEAD1;
    output[2] = frame->version;
    output[3] = frame->type;
    output[4] = (u8)frame->sequence;
    output[5] = (u8)(frame->sequence >> 8);
    output[6] = (u8)frame->length;
    output[7] = (u8)(frame->length >> 8);
    if (frame->length) memcpy(&output[8], frame->payload, frame->length);
    crc = ble_crc16_modbus(&output[2], 6 + frame->length);
    output[8 + frame->length] = (u8)crc;
    output[9 + frame->length] = (u8)(crc >> 8);
    return total;
}
