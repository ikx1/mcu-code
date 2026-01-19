/**
 * @file ws2812_task.c
 * @author 
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "ws2812_task.h"
#include "bsp_ws2812_driver.h"
#include "control_task.h"
#include "bsp_battery_handler.h"

#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/
#define TURN_TWINKLE_FREQ   20
#define EM_TWINKLE_FREQ   40

/********************************** Variables ********************************/


/********************************** Functions ********************************/
void WS2812_Task(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();

    while(1)
    {
		const ROBOT_MODE_T* robot_mode = get_robot_mode_p();
		
		if(robot_mode->robot_mode_main == MODE_EMERGENCY)
		{
			ws2811_all_same_color(RGB_RED);
		}
		
		else if(robot_mode->robot_mode_main == MODE_READY)
		{
			ws2811_all_same_color(RGB_YELLOW);
		}
	
		else if(robot_mode->robot_mode_main == MODE_REMOTE)
		{
			if (robot_mode->robot_mode_sub == SUB_SLAVE)
			{
				ws2811_all_same_color(RGB_BLUE);
			}
			else
			{
				ws2811_all_same_color(RGB_GREEN);
			}					
		}		

		ws2811_show();
		
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50)); 	 
    }
}  
