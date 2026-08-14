#ifndef __USART3_H__
#define __USART3_H__

/** @file usart3.h @brief ESP8266 AT 通信串口及接收环形缓冲接口。 */
#include "sys.h"

#define UART3_RX_RING_SIZE 1024U /* 为较长的 MQTT 订阅行预留接收空间。 */

void usart3_init(u32 bound);
void u3_printf(char *fmt, ...);
void uart3_write(const u8 *data, u16 length);
u8 uart3_rx_pop(u8 *byte);
u32 uart3_rx_overflow_count(void);

#endif
