/**
 * @file control_task.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


/********************************** Defines **********************************/
#define TURN_M      			(1091)

#define DEADZONE 				(8)
#define INPUT_MAX               100 

#define MAX_INPUT_VALUE    		100        // 遥控器最大输入值
#define MIN_INPUT_VALUE    		-100      // 遥控器最小输入值

/********************************** Variables ********************************/
typedef enum {
    CONTROL_OK = 0,                  // 成功
    CONTROL_ERR,                     // 失败
    CONTROL_ERR_PARMETER,
    CONTROL_NULL_POINTER,           // motor_driver_t 或 info 指针为空
    CONTROL_NOT_INITIALIZED,        // 电机未初始化
    CONTROL_ERR_UNKNOWN_TYPE,           // 不支持的电机类型
    CONTROL_NO_CONTROL_FUNCTION,    // 控制函数指针为空
} control_err_t;

/* 主控制模式 */
typedef enum 
{
    MODE_READY = 0,
    MODE_EMERGENCY,
    MODE_REMOTE,
    MODE_DISPLAY,
    MODE_UNKNOWN,
} ROBOT_MODE_MAIN;

/* 子控制模式 */
typedef enum 
{
    SUB_STOP = 0,
    SUB_MANUAL,
    SUB_SLAVE,
    SUB_CHARGE,
    SUB_MOTOR_CTRL,
    SUB_MOTOR_ERR_CLR,
    SUB_UNKNOWN,
} ROBOT_MODE_SUB;

/* 速度档位 */
typedef enum 
{
    SPEED_LOW = 0,
    SPEED_HIGH
} ROBOT_SPEED_LEVEL;

typedef struct
{
    ROBOT_MODE_MAIN robot_mode_main;
    ROBOT_MODE_SUB robot_mode_sub;
    ROBOT_SPEED_LEVEL robot_speed_level;
}ROBOT_MODE_T;

typedef struct
{
    float high_speed;       // 高速线速度上限（m/s）
    float high_roat;        // 高速角速度上限（rad/s）
    float low_speed;        // 低速线速度上限（m/s）
    float low_roat;         // 低速角速度上限（rad/s）
}robot_speed_par_t;

typedef struct
{
    int32_t glob_line_speed;
    int32_t glob_rota_speed;
    int32_t glob_joint_pos;

    uint16_t status_word;

    uint16_t time_stamp;
}ROBOT_INFO;

/********************************** Functions ********************************/
void ContronlTask(void *pvParameters);

const ROBOT_MODE_T* get_robot_mode_p(void);

ROBOT_INFO* get_robot_p(void);
void robot_info_snapshot(ROBOT_INFO *out);


#endif /* __CONTROL_TASK_H__ */
