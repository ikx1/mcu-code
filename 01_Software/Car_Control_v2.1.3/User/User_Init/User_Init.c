/**
 * @file User_Init.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include <stdio.h>

#include "User_Init.h"
#include "motor_task.h"
#include "uart_task.h"
#include "ibus_task.h" 
#include "control_task.h"
#include "io_task.h"
#include "battery_task.h"
#include "wdt_task.h"
#include "ws2812_task.h"
#include "uart_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "can_queue.h"
#include "bsp_uart_driver.h"
#include "bsp_ibus_handler.h"
#include "bsp_battery_driver.h"
#include "bsp_battery_handler.h"
#include "bsp_handler_display.h"

#include "uart_rtos.h"
#include "uart_proto_newfmt.h"

#include "system_cfg.h"

/********************************** Defines **********************************/
#define TASK_PRIO_USER_INIT	26
#define TASK_PRIO_SCRAM		25
#define TASK_PRIO_CONTROL	24
#define TASK_PRIO_CAN_RX	23
#define TASK_PRIO_CAN_SVC	22
#define TASK_PRIO_MOTOR		21
#define TASK_PRIO_SBUS		21
#define TASK_PRIO_MODBUS	20
#define TASK_PRIO_WDT		19
#define TASK_PRIO_WS2812	18

/********************************** Variables ********************************/
TaskHandle_t userTaskInitHandle = NULL;//任务句柄

#define HARDWARE_VERSION	"V1.0.0"
#define SOFTWARE_VERSION  	"V0.1.0"

/********************************** Functions ********************************/
void userTaskInitFunction(void *pvParameters)
{			
	xTaskCreate(ScramTask, 		  "scram", 		  128,   NULL, TASK_PRIO_SCRAM, NULL);
	xTaskCreate(ContronlTask, 	  "contronl", 	  128*5, NULL, TASK_PRIO_CONTROL, NULL);	
	xTaskCreate(Motor_Task, 	  "motor", 		  128*4, NULL, TASK_PRIO_MOTOR, NULL);
	xTaskCreate(Robot_Speed_Task, "filter_speed", 128*2, NULL, 21, NULL);
	xTaskCreate(Can_Analy_Task,   "can_analysis", 	  128*4, NULL, TASK_PRIO_CAN_RX, NULL);
	xTaskCreate(can_send_task, "can_send", 128*2, NULL, TASK_PRIO_CAN_SVC, NULL);
	
	xTaskCreate(ibusTask,		  "ibus", 		  128*2, NULL, TASK_PRIO_SBUS, NULL);
	xTaskCreate(ModbusTask,    "modbus",   		  128*4, NULL, TASK_PRIO_MODBUS, NULL); 
	xTaskCreate(WS2812_Task, 	"ws2812", 		  128*4, NULL, TASK_PRIO_WS2812, NULL);
	
#ifndef DEBUG
	xTaskCreate(wdt_monitor_thread, "wdt_monitor", 128*2, NULL, TASK_PRIO_WDT, NULL);	
#endif /* DEBUG */	
//	uint16_t tem = xPortGetFreeHeapSize();

	vTaskDelete(NULL);//删除任务       
}


void UserAppTask_Init(void)
{
	BaseType_t xReturn = pdPASS;
	
	can_queue_init();
	motor_register();
	
	uart_task_protocol_init();

	uart_rtos_cfg_t cfg1 = {
        .uart_id       = DEV_UART1,
        .tx_period_ms  = 20,
        .rx_period_ms  = 20,
        .rx_chunk_max  = 256,
        .build_tx      = app_uart_build_tx,
        .on_rx         = app_uart_on_rx,
        .user_ctx      = NULL,
        .tx_task_stack_words = 256,
        .rx_task_stack_words = 256,
        .tx_task_prio  = 20,
        .rx_task_prio  = 21,
    };
    (void)uart_rtos_start(&cfg1);
	
	ibus_driver_init();
	ibus_driver_register_callback(ibus_callback);
	
	modbus_driver_register_callback(modbus_callback);
	
	display_driver_register_callback(display_callback);

#ifndef DEBUG
	wdt_monitor_init();
#endif /* DEBUG */		
    
	xReturn = xTaskCreate(userTaskInitFunction, "userTask",	128 * 4, NULL, 26, &userTaskInitHandle);
	
	if(xReturn)
	{
		
	}
		
}
