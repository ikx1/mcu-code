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
#include <stdbool.h>
#include <stdint.h>
#include "bsp_motor_handler.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef enum
{
    MOTOR_JOINT_HOME_IDLE = 0,
    MOTOR_JOINT_HOME_WAIT_MOTOR,
    MOTOR_JOINT_HOME_SEEK_LOWER,
    MOTOR_JOINT_HOME_SETTLE,
    MOTOR_JOINT_HOME_ZERO,
    MOTOR_JOINT_HOME_DONE,
    MOTOR_JOINT_HOME_FAULT,
} motor_joint_home_state_t;

typedef struct
{
    uint8_t request_enable;
    uint8_t need_home;
    uint8_t homed;
    uint8_t busy;
    uint8_t fault;
    motor_joint_home_state_t state;
} motor_joint_home_status_t;


/********************************** Functions ********************************/
void motor_task_joint_home_request_set(bool enable);
void motor_task_joint_home_status_snapshot(motor_joint_home_status_t *out);
bool motor_task_joint_is_homed(void);
bool motor_task_joint_home_is_busy(void);

void Can_Analy_Task(void *pvParameters);

void Motor_Task(void *pvParameters);
void Robot_Speed_Task(void *pvParameters);




#endif /* __MOTOR_TASK_H__ */
