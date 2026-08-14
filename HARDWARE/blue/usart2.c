/**
 * @file usart2.c
 * @brief BLE 链路驱动：ISR 生产、线程消费的单生产者单消费者环形缓冲。
 */
#include "usart2.h"
#include "ble_service.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

static volatile u8 ble_rx_ring[BLE_UART_RX_RING_SIZE];
static volatile u16 ble_rx_head;
static volatile u16 ble_rx_tail;
static volatile u32 ble_rx_overflow;

void Ble_IoInit(void)
{
    /* PG13 配置为推挽输出并拉高，使 BLE 模块进入工作状态。 */
    GPIO_InitTypeDef gpio;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOG, &gpio);
    BLE_WKUP = 1;
}

void ble_uart_rx_init(void)
{
    /* head 只由 ISR 推进，tail 只由 blerx 线程推进。 */
    ble_rx_head = 0;
    ble_rx_tail = 0;
    ble_rx_overflow = 0;
}

void ble_uart_rx_push_isr(u8 byte)
{
    u16 next = (u16)((ble_rx_head + 1U) % BLE_UART_RX_RING_SIZE);
    if (next == ble_rx_tail)
    {
        /* 满时不能在中断中等待，丢弃新字节并由协议 CRC 发现坏帧。 */
        ++ble_rx_overflow;
        return;
    }
    ble_rx_ring[ble_rx_head] = byte;
    ble_rx_head = next;
}

u8 ble_uart_rx_pop(u8 *byte)
{
    if (byte == RT_NULL || ble_rx_tail == ble_rx_head) return 0;
    *byte = ble_rx_ring[ble_rx_tail];
    ble_rx_tail = (u16)((ble_rx_tail + 1U) % BLE_UART_RX_RING_SIZE);
    return 1;
}

u32 ble_uart_rx_overflow_count(void)
{
    return ble_rx_overflow;
}

void ble_uart_write(const u8 *data, u16 length)
{
    u16 i;
    if (data == RT_NULL) return;
    /* 阻塞发送只允许在工作线程中调用，禁止在接收 ISR 内调用。 */
    for (i = 0; i < length; ++i)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, data[i]);
    }
}

void usart2_init(u32 bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    /* PA2=TX、PA3=RX，8 位数据、1 位停止、无校验、无流控。 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate = bound;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    /* ISR 仅搬运字节，因此使用较低抢占优先级。 */
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    ble_uart_rx_init();
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        /* 读取数据寄存器、写入环形缓冲，然后唤醒协议解析线程。 */
        ble_uart_rx_push_isr((u8)USART_ReceiveData(USART2));
        ble_service_notify_rx_isr();
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
