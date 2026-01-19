/**
 * @file motor_task.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include "bsp_motor_handler.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef struct 
{
    float wheel_speed_left;
    float wheel_speed_right;

    int32_t global_speed;
    int32_t global_roation;
}car_info_t;


/********************************** Functions ********************************/
const car_info_t* get_car_info_p(void);

void Can_Analy_Task(void *pvParameters);
void can_send_task(void *pvParameters);

void Motor_Task(void *pvParameters);
void Robot_Speed_Task(void *pvParameters);




#endif /* __MOTOR_TASK_H__ */
