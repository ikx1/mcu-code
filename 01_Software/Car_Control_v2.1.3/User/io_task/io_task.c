/**
 * @file io_task.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "io_task.h"

#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
void ScramTask(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		input_driver_update_10ms();
			
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
	}
}
