/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-24                  the first version
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
	 
#include "Task.h"
#include "usart3.h"
#include "usart2.h"
#include "usart.h"

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
/*
 * Please modify RT_HEAP_SIZE if you enable RT_USING_HEAP
 * the RT_HEAP_SIZE max value = (sram size - ZI size), 1024 means 1024 bytes
 */
#define RT_HEAP_SIZE (30*1024)
static rt_uint8_t rt_heap[RT_HEAP_SIZE];

RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

void rt_os_tick_callback(void)
{
    rt_interrupt_enter();
    
    rt_tick_increase();

    rt_interrupt_leave();
}

/**
 * This function will initial your board.
 */
void rt_hw_board_init(void)
{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
		uint32_t msCnt;     // count value of 1ms
		SystemCoreClockUpdate();
    msCnt = SystemCoreClock / RT_TICK_PER_SECOND;
		SysTick_Config(msCnt);
//#error "TODO 1: OS Tick Configuration."
    /* 
     * TODO 1: OS Tick Configuration
     * Enable the hardware timer and call the rt_os_tick_callback function
     * periodically with the frequency RT_TICK_PER_SECOND. 
     */

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
	
  
#ifdef AT24_Write_data 

#endif


#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

#ifdef RT_USING_CONSOLE

static int uart_init(void)
{
//#error "TODO 2: Enable the hardware uart and config baudrate."
		usart_init(115200);
		usart2_init(115200);
		usart3_init(115200);
    return 0;
}
INIT_BOARD_EXPORT(uart_init);

void rt_hw_console_output(const char *str)
{
//#error "TODO 3: Output the string 'str' through the uart."
	
	 /* 进入临界段 */  
	 //禁止操作系统的调度，进入临界段的代码不允许打断，当rt_scheduler_lock_nest>=1时，调度器停止调度。 
  rt_enter_critical();
//	printf("%s\n",str);
	while(*str!='\0')
	{
		 /* 换行 */
		if (*str == '\n')//RT-Thread 系统中已有的打印均以 \n 结尾，而并非 \r\n，所以在字符输出时，需要在输出 \n 之前输出 \r，完成回车与换行，否则系统打印出来的信息将只有换行
		{
		    USART_SendData(USART3, '\r');
			while(USART_GetFlagStatus(USART3, USART_FLAG_TC)== RESET);
		}
		USART_SendData(USART3, *(str++));
	    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE)== RESET);	 //该标志位使用TC时会导致finsh组件有出错的可能
	}
	 /* 退出临界段 */  
  rt_exit_critical();  //注意：使用进入临界段语句rt_enter_critical(); 一定要使用退出临界段语句 rt_exit_critical();否则调度器锁住，无法进行调度
}
#endif

/* cortex-m 架构使用 SysTick_Handler() */
void SysTick_Handler()
{
    rt_os_tick_callback();
}

/*finsh控制台作为一个线程也在不断的执行,默认优先级为21,主线程的函数优先级需大于21(数字小于21),否则会无法响应主线程*/
char rt_hw_console_getchar(void)
{
    /* Note: the initial value of ch must < 0 */
    int ch = -1;
 
//#error "TODO 4: Read a char from the uart and assign it to 'ch'."
		
		if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
		{
			ch = (char)USART_ReceiveData(USART1);
		}
		else
		{
			if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
			{
				USART_ClearFlag(USART1, USART_FLAG_TC);
			}
		}
    return ch;
}

