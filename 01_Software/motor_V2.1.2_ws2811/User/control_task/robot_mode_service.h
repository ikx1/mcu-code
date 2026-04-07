#ifndef __ROBOT_MODE_SERVICE_H__
#define __ROBOT_MODE_SERVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MODE_READY = 0,
    MODE_EMERGENCY,
    MODE_REMOTE,
    MODE_DISPLAY,
    MODE_UNKNOWN,
} ROBOT_MODE_MAIN;

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
} ROBOT_MODE_T;

void robot_mode_snapshot(ROBOT_MODE_T *out);

#ifdef __cplusplus
}
#endif

#endif /* __ROBOT_MODE_SERVICE_H__ */
