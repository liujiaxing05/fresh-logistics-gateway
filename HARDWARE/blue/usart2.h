#ifndef __USART2_H__
#define __USART2_H__

/** @file usart2.h @brief 下层 BLE 透明传输串口及接收环形缓冲接口。 */
#include "sys.h"

#define BLE_UART_RX_RING_SIZE 512U /* 实际可存 511 B，空槽用于区分满与空。 */
#define BLE_WKUP PGout(13)         /* BLE 模块唤醒/使能引脚。 */

void usart2_init(u32 bound);
void Ble_IoInit(void);
void ble_uart_rx_init(void);
void ble_uart_rx_push_isr(u8 byte);
/** 从线程上下文弹出一个字节；成功返回 1，空缓冲返回 0。 */
u8 ble_uart_rx_pop(u8 *byte);
/** 获取环形缓冲已满时累计丢弃的字节数。 */
u32 ble_uart_rx_overflow_count(void);
void ble_uart_write(const u8 *data, u16 length);

#endif
