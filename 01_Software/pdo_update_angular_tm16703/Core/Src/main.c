/**
 * @file main.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "main.h"
#include "uart.h"
#include "delay.h"
#include "uart.h"
#include "gpio.h"
#include "tim.h"
#include "iwdg.h"
#include "User_Init.h"
#include "FreeRTOS.h"
#include "task.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/
volatile TaskHandle_t g_stack_overflow_task_handle = NULL;
volatile const char *g_stack_overflow_task_name = NULL;


/********************************** Functions ********************************/


int main(void)
{   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 	// 	配置中断函数组
	
	delay_init();
		
	
	gpio_init();
	
	uart1_init();
//	uart2_init();
 	uart3_init();
 	uart4_init();						//串口4，初始化配置
	uart5_init();
	
	WS2811_TIM3_CH2_PWM_INIT();
	
	UserAppTask_Init();

	for(;;);
}


//任务栈区溢出钩子函数
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    g_stack_overflow_task_handle = xTask;
    g_stack_overflow_task_name = pcTaskName;
    taskDISABLE_INTERRUPTS();
	for(;;);
}
