/**
 * @file ibus_task.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "ibus_task.h" 
#include "bsp_ibus_handler.h"
#include "bsp_ibus_driver.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/

/********************************** Functions ********************************/
void ibusTask(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	// 初始化 handler（设定失联超时 100ms）
	IBUS_Handler* ibus_data = get_ibus_data_p();
    ibus_handler_init(ibus_data, 100);
		
    while (1)
    {
		uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

		ibus_data = get_ibus_data_p();
		
        // 失联检测（更新 connected 状态）
        ibus_handler_check_timeout(ibus_data, now_ms);

        if (!ibus_data->connected) 
        {
            ibus_handler_receive_timeout(ibus_data);
        } 

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
	}
}

