/**
 * @file main.c
 * @brief 生鲜储运网关应用入口。
 *
 * RT-Thread 完成内核和板级初始化后调用本函数。本层只初始化产品硬件，
 * 长期运行的业务由 TaskInit() 创建的线程和定时器负责；main() 返回后
 * RT-Thread 调度器仍会继续运行。
 */
#include "Task.h"
#include "lcd.h"
#include "beep.h"
#include "led.h"
#include "usart2.h"


int main(void)
{
    /* 提前初始化并清屏，后续由 display 线程周期刷新业务状态。 */
    LCD_Init();
    LCD_Clear(WHITE);

    /* 蜂鸣器与 LED 是本地告警输出，统一由 alarm 线程控制。 */
    BEEP_Init();
    LED_Init();

    /* PG13 唤醒 BLE 模块，USART2 提供与下层设备的透明串口链路。 */
    Ble_IoInit();
    
    /* 创建通信服务、业务线程以及周期遥测/心跳定时器。 */
    TaskInit();
    return 0;
}
