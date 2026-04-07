/**
 * @file bsp_can_driver.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-05-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_MOTOR_DRIVER_H__
#define __BSP_MOTOR_DRIVER_H__

/********************************** Includes *********************************/
#include <stdio.h>
#include <stdint.h>

#include "system_cfg.h"

/********************************** Defines **********************************/
#define  SDO_W1   0x2F
#define  SDO_W2   0x2B
#define  SDO_W4   0x23
#define  SDO_RD   0x40

#define INITED  1
#define NOT_INITED  0


/********************************** Variables ********************************/
typedef struct motor_driver motor_driver_t;

typedef enum {
	/* 两驱动：RPDO1=Controlword，RPDO2=Target velocity，TPDO1=statusword(16)+position(32) */
	SERVO_SDO_LAYOUT_DRIVE_VEL = 0,
	/* 滑台：RPDO1=Controlword+Target position，TPDO1=status+vel+pos */
	SERVO_PDO_LAYOUT_SLIDE_POS,
} servo_pdo_layout_t;

typedef enum {
    MOTOR_STATE_UNKNOWN = 0x0000,
    MOTOR_STATE_READING = 0x0037,
    MOTOR_STATE_POWER_ON = 0x0050,
    MOTOR_STATE_READY = 0x0031,
    MOTOR_STATE_DISABLED = 0x0033,
    MOTOR_STATE_ALARM = 0x0088,
    JOINT_READY = 0x1050,
    JOINT_POWER_ON = 0x1031,
    JOINT_ENABLE = 0x0437,
    JOINT_ALARM = 0x0098,
} motor_state_t;

typedef enum
{
    ERR_OK = 0x0000,
	ERR_ENCODER_FAULT_ABZ = 0x0001, //编码器故障 ABZ 报警
	ERR_ENCODER_FAULT_UVW = 0x0002, //编码器故障 UVW 报警
	ERR_POSITION_OVERSHOOT = 0x0003, //位置超差
	ERR_STALL= 0x0004, //失速
	ERR_CURRENT_SAMPLING_FAULT = 0x0005, //电流采样故障
	ERR_OVER_LOAD = 0x0006, //过载
	ERR_UNDER_VOLTAGE = 0x0007, //欠压
	ERR_OVER_VOLTAGE = 0x0008, //过压
	ERR_OVER_CURRENT = 0x0009, //过流
	ERR_DISCHARGE_INSTANTANEOUS_POWER = 0x000A, //放电瞬时功率过大
	ERR_DISCHARGE_MEAN_POWER = 0x000B, //放电平均功率大
	ERR_PARAMETER_RW_EXCEPTION = 0x000C, //参数读写异常
	ERR_FUNCTION_REPEAT = 0x000D, //输入口功能定义重复
	ERR_WATCHDOG_TRIGGER = 0x000E, //通讯看门狗触发
	ERR_MOTOR_TOO_WARM = 0x000F, //电机过温报警
}motor_err_t;

typedef enum{
    MOTOR_INIT_NMT_PREOP = 0,   // 1) NMT Pre-Operational
    MOTOR_INIT_PDO_DISABLE,     // 2) 禁用PDO + 清映射
    MOTOR_INIT_SET_MODE,        // 3) 配模式
    MOTOR_INIT_SET_PROFILE,     // 4) 写0x6083/0x6084等参数
    MOTOR_INIT_RPDO_CFG,        // 5) 映射RPDO
    MOTOR_INIT_TPDO_CFG,        // 6) 映射TPDO
    MOTOR_INIT_NMT_OP,          // 7) NMT Operational
    MOTOR_INIT_DONE,            // done
} motor_init_type_t;

typedef struct
{
    motor_init_type_t init_type;
    uint8_t initialized; /* INITED/NOT_INITED */
    uint8_t ready_flag;
} motor_init_t;

typedef struct 
{
    uint32_t id;
    motor_state_t state;
    motor_err_t state_err;
    int32_t current_speed;
    float wheel_speed;
    float wheel_linear;
    float wheel_angular;

    int32_t set_speed;
    int32_t current_position;
    float location;
    int32_t last_position;
    int32_t set_position;
    uint8_t initialized;
} motor_info_t;

typedef struct motor_driver 
{
    motor_info_t *info;
    motor_init_t init_state;
    servo_pdo_layout_t pdo_layout;

    void (*init)(motor_driver_t *self);
    void (*enter_ready)(motor_driver_t *self);
    void (*enable)(motor_driver_t *self);
    void (*disable)(motor_driver_t *self);
    void (*set_speed)(motor_driver_t *self, int32_t rpm);
    void (*set_relative_position)(motor_driver_t *self, int32_t pos);
    void (*set_absolute_position)(motor_driver_t *self, int32_t pos);

    void (*inquire_speed)(motor_driver_t *self);
    void (*inquire_encoder)(motor_driver_t *self);
    void (*inquire_state)(motor_driver_t *self);
    void (*inquire_state_err)(motor_driver_t *self);
    
    void (*update_status)(motor_driver_t *self, uint16_t new_state);
    void (*update_state_err)(motor_driver_t *self, uint16_t new_state);
    void (*update_encoder)(motor_driver_t *self, int32_t pos);
    void (*update_speed)(motor_driver_t *self, int32_t rpm);

    void (*reset_zero)(motor_driver_t *self);
    void (*reset_motor)(motor_driver_t *self);
} motor_driver_t;

/********************************** Functions ********************************/
void motor_driver_create(motor_driver_t *driver, motor_info_t *info);


#endif /* __BSP_CAN_DRIVER_H__ */
