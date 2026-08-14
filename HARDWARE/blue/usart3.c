/**
 * @file usart3.c
 * @brief ESP8266 串口驱动及 ISR/线程之间的接收环形缓冲。
 */
#include "usart3.h"
#include "cloud_service.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdarg.h>
#include <stdio.h>

static volatile u8 uart3_rx_ring[UART3_RX_RING_SIZE];
static volatile u16 uart3_rx_head;
static volatile u16 uart3_rx_tail;
static volatile u32 uart3_rx_overflow;

void uart3_write(const u8 *data, u16 length)
{
    u16 i;
    if (data == RT_NULL) return;
    /* 上层通过 mqtttx 或配网流程串行调用，避免 AT 命令交叉。 */
    for (i = 0; i < length; ++i)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        USART_SendData(USART3, data[i]);
    }
}

void u3_printf(char *fmt, ...)
{
    char buffer[256];
    int length;
    va_list args;
    va_start(args, fmt);
    /* 固定栈缓冲避免格式化期间使用动态内存。 */
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    if (length > 0)
    {
        if (length >= (int)sizeof(buffer)) length = sizeof(buffer) - 1;
        uart3_write((const u8 *)buffer, (u16)length);
    }
}

u8 uart3_rx_pop(u8 *byte)
{
    /* 在线时由 cloudrx 消费；配网时由 wifista 同步消费 AT 响应。 */
    if (byte == RT_NULL || uart3_rx_tail == uart3_rx_head) return 0;
    *byte = uart3_rx_ring[uart3_rx_tail];
    uart3_rx_tail = (u16)((uart3_rx_tail + 1U) % UART3_RX_RING_SIZE);
    return 1;
}

u32 uart3_rx_overflow_count(void)
{
    return uart3_rx_overflow;
}

void usart3_init(u32 bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    /* PD8=TX、PD9=RX；GPIO 端口必须对应开启 GPIOD 时钟。 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    usart.USART_BaudRate = bound;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &usart);

    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    uart3_rx_head = uart3_rx_tail = 0;
    uart3_rx_overflow = 0;
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        u16 next = (u16)((uart3_rx_head + 1U) % UART3_RX_RING_SIZE);
        u8 byte = (u8)USART_ReceiveData(USART3);
        /* 环形缓冲满时丢弃新字节；ISR 不能等待线程消费。 */
        if (next == uart3_rx_tail) ++uart3_rx_overflow;
        else
        {
            uart3_rx_ring[uart3_rx_head] = byte;
            uart3_rx_head = next;
            /* 仅通知“有数据”，AT 行和 JSON 解析均在线程上下文执行。 */
            cloud_service_notify_rx_isr();
        }
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
