/**
 * @file bsp_can_driver.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-05-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_motor_driver.h"
#include "canopen_proto.h"
#include "system_cfg.h"

#include <stddef.h>

/********************************** Defines **********************************/
#define SERVO_IDX_RPDO1_COMM 0x1400u
#define SERVO_IDX_RPDO2_COMM 0x1401u
#define SERVO_IDX_TPDO1_COMM 0x1800u
#define SERVO_IDX_TPDO2_COMM 0x1801u

#define SERVO_IDX_RPDO1_MAP  0x1600u
#define SERVO_IDX_RPDO2_MAP  0x1601u
#define SERVO_IDX_TPDO1_MAP  0x1A00u
#define SERVO_IDX_TPDO2_MAP  0x1A01u

#define SERVO_RPDO_MAP_CONTROL        0x60400010u
#define SERVO_RPDO_MAP_TARGET_SPEED   0x60FF0020u
#define SERVO_RPDO_MAP_TARGET_POS     0x607A0020u  /* 你原来叫 SERVO_TPDO_MAP_TARGET_POS，实为目标位置 */

#define SERVO_TPDO_MAP_STATUS         0x60410010u
#define SERVO_TPDO_MAP_ACTUAL_POS     0x60630020u
#define SERVO_TPDO_MAP_ACTUAL_SPEED   0x60690020u

/* ---- 默认 COB-ID（按你的实际配置可改）---- */
#define SERVO_COB_RPDO1(id)   (0x200u + (id))
#define SERVO_COB_RPDO2(id)   (0x300u + (id))
#define SERVO_COB_TPDO1(id)   (0x180u + (id))
#define SERVO_COB_TPDO2(id)   (0x280u + (id))

#define PDO_DISABLE_MASK      (0x80000000UL)


/********************************** Variables ********************************/


/********************************** Functions ********************************/
/**
 * @brief 禁用并清除电机PDO
 * 
 * @param id 电机ID
 */
static void motor_pdo_disable_and_clear(uint8_t id)
{
    /* 1) 先禁用 PDO（COB-ID bit31=1），同时写入你期望的 COB-ID（更稳） */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO1(id));
    SDO_Write_Data4(id, SERVO_IDX_TPDO2_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO2(id));

    /* 2) 清映射：sub0=0（标准 remap 第一步） */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x00);
    SDO_Write_Data1(id, SERVO_IDX_TPDO2_MAP, 0x00, 0x00);
}

/**
 * @brief 配置电机TPDO1
 * 
 * @param id 电机ID
 */
static void motor_cfg_tpdo1(uint8_t id)
{
    /* A) 禁用（你原来就做了，这里保留并规范成“写期望COB-ID|disable”） */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO1(id));

    /* B) 通信参数 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_COMM, 0x02, 0xFE);   // transmission type
    SDO_Write_Data2(id, SERVO_IDX_TPDO1_COMM, 0x03, 0x0001); // inhibit time
    SDO_Write_Data2(id, SERVO_IDX_TPDO1_COMM, 0x05, 0x000a); // event timer

    /* C) remap：先0 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x00);

    /* D) 写映射项 */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x01, SERVO_TPDO_MAP_STATUS);
    // SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x02, SERVO_TPDO_MAP_ACTUAL_POS);
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x02, SERVO_TPDO_MAP_ACTUAL_SPEED);

    /* E) 写映射数量 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x02);

    /* F) 启用 */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, SERVO_COB_TPDO1(id));
}

/**
 * @brief 初始化电机驱动程序
 * 
 * @param self 电机驱动程序结构体指针
 */
static void __motor_driver_real_init(motor_driver_t *self)
{
    uint8_t id;

    if ((self == NULL) || (self->info == NULL))
    {
        return;
    }

    id = (uint8_t)self->info->id;

    /* The slide axis reports status/position through its own TPDO stream, so
     * init only needs to push the node into operational once. */
    if (self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        if (self->init_state.initialized != INITED)
        {
            CANopen_NMT(0x01, id);
            self->init_state.initialized = INITED;
        }
        return;
    }

    if (self->init_state.initialized == INITED) return;

    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        switch(self->init_state.init_type)
        {
            case MOTOR_INIT_NMT_PREOP:
                /* 1) 先拉到 Pre-Op，确保允许做 PDO/通信参数配置 */
                CANopen_NMT(0x80, id);
                self->init_state.init_type = MOTOR_INIT_PDO_DISABLE;
                break;

            case MOTOR_INIT_PDO_DISABLE:
                /* 2) 禁用/清PDO，避免“TPDO抢跑/OP下禁止映射” */
                motor_pdo_disable_and_clear(id);
                self->init_state.init_type = MOTOR_INIT_SET_MODE;
                break;

            case MOTOR_INIT_SET_MODE:
                /* 3) 设置控制模式 */
                Control_Mode_SET(id, 3);
                self->init_state.init_type = MOTOR_INIT_SET_PROFILE;
                break;

            case MOTOR_INIT_SET_PROFILE:
                /* 4) 参数配置 */
                SDO_Write_Data4(id, 0x6083, 0x00, 0x07D0); // 加速时间
                SDO_Write_Data4(id, 0x6084, 0x00, 0x07D0); // 减速时间
                self->init_state.init_type = MOTOR_INIT_RPDO_CFG;
                break;

            case MOTOR_INIT_RPDO_CFG:
                /* 5) RPDO映射（标准顺序） */
                // motor_cfg_rpdo1(id);
                // motor_cfg_rpdo2(id);
                self->init_state.init_type = MOTOR_INIT_TPDO_CFG;
                break;

            case MOTOR_INIT_TPDO_CFG:
                /* 6) TPDO映射（标准顺序） */
                motor_cfg_tpdo1(id);
                self->init_state.init_type = MOTOR_INIT_NMT_OP;
                break;

            case MOTOR_INIT_NMT_OP:
                /* 7) 进入 Operational，后续再走 0x06/0x07/0x0F */
                CANopen_NMT(0x01, id);
                self->init_state.init_type = MOTOR_INIT_DONE;
                break;

            case MOTOR_INIT_DONE:
                self->init_state.initialized = INITED;
                break;

            default:
                break;
        }
    }
}

/**
 * @brief 进入就绪状态
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_enter_ready(motor_driver_t *self)
{	
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data2(self->info->id, 0x6040, 0x00, 0x06);
    }
}

/**
 * @brief 启用电机
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_enable(motor_driver_t *self) 
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data4(self->info->id, 0x6040, 0x00, 0x000F);  // Enable
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        PDO_Write_Data6(self->info->id, 0x07, 0x0000);
    }
}

/**
 * @brief 禁用电机
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_disable(motor_driver_t *self) 
{
    if(self->init_state.initialized != INITED) return;

    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data4(self->info->id, 0x6040, 0x00, 0x0007);  // Disable
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        PDO_Write_Data6(self->info->id, 0x06, 0x0000);
    }
}

/**
 * @brief 设置电机速度
 * 
 * @param self 电机驱动程序结构体指针
 * @param rpm 速度值（单位：转/分）
 */
static void motor_driver_set_speed(motor_driver_t *self, int32_t rpm) 
{
    if(self->init_state.initialized != INITED) return;

    self->info->set_speed = rpm;
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data4(self->info->id, 0x60ff, 0x00, (int32_t)rpm);
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        
    }
}

/**
 * @brief 设置电机相对位置
 * 
 * @param self 电机驱动程序结构体指针
 * @param pos 位置值（单位：度）
 */
static void motor_driver_set_relative_position(motor_driver_t *self, int32_t pos) 
{
    if(self->init_state.initialized != INITED) return;

    self->info->set_position = pos;
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data4(self->info->id, 0x607A, 0x00, pos);
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        /* Motion feedback comes from the drive's TPDO stream; keep manual jog
         * traffic to pure PDO writes here. */
        PDO_Write_Data6(self->info->id, 0x0f, pos);
        PDO_Write_Data6(self->info->id, 0x1f, pos);
    }
}

/**
 * @brief 设置电机绝对位置
 * 
 * @param self 电机驱动程序结构体指针
 * @param pos 位置值（单位：度）
 */
static void motor_driver_set_absolute_position(motor_driver_t *self, int32_t pos) 
{
    if(self->init_state.initialized != INITED) return;

    self->info->set_position = pos;
    if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        /* Host/manual point commands also rely on TPDO feedback, so avoid any
         * synchronous SDO status/position reads on the command path. */
        PDO_Write_Data6(self->info->id, 0x0f, pos);
        PDO_Write_Data6(self->info->id, 0x5f, pos);
    }
}

/**
 * @brief 查询电机速度
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_inquire_speed(motor_driver_t *self)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Read_Data4(self->info->id, 0x6069, 0x00);
    }
}

/**
 * @brief 查询电机编码器值
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_inquire_encoder(motor_driver_t *self)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Read_Data4(self->info->id, 0x6063, 0x00);
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        SDO_Read_Data4((uint8_t)self->info->id, 0x6063u, 0x00u);
    }
}

/**
 * @brief 查询电机状态
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_inquire_state(motor_driver_t *self)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Read_Data4(self->info->id, 0x6041, 0x00);
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        SDO_Read_Data4((uint8_t)self->info->id, 0x6041u, 0x00u);
    }
}

static void motor_driver_inquire_state_err(motor_driver_t *self)
{
    if ((self == NULL) || (self->info == NULL))
    {
        return;
    }

    if (self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Read_Data4((uint8_t)self->info->id, 0x603Fu, 0x00u);
    }
}

/**
 * @brief 更新电机速度
 * 
 * @param self 电机驱动程序结构体指针
 * @param rpm 速度值（单位：转/分）
 */
static void motor_driver_update_speed(motor_driver_t *self, int32_t rpm)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        self->info->current_speed = rpm / DRIVE_GEAR_RATIO;
        self->info->wheel_angular = RPM_TO_ANGULAR(self->info->current_speed);
        self->info->wheel_linear = self->info->wheel_angular * WHEEL_RADIUS;
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        self->info->current_speed = rpm;
    }
}

/**
 * @brief 更新电机编码器值
 * 
 * @param self 电机驱动程序结构体指针
 * @param val 编码器值
 */
static void motor_driver_update_encoder(motor_driver_t *self, int32_t val)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        self->info->last_position = self->info->current_position;
        self->info->current_position = val;
        self->info->wheel_speed = (self->info->current_position - 
                                                     self->info->last_position) 
                                               * 20 / 5600.0 * WHEEL_PERIMETER;
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        self->info->current_position = val;
        self->info->location = SYSTEM_CFG_JOINT_POS_TO_MM(val);
    }
}

/**
 * @brief 更新电机状态
 * 
 * @param self 电机驱动程序结构体指针
 * @param state 电机状态值
 */
static void motor_driver_update_state(motor_driver_t *self, uint16_t state)
{
    self->info->state = (motor_state_t)state;
    if ((state != (uint16_t)MOTOR_STATE_ALARM) &&
        (state != (uint16_t)JOINT_ALARM))
    {
        self->info->state_err = ERR_OK;
    }
}

/**
 * @brief 更新电机状态错误
 * 
 * @param self 电机驱动程序结构体指针
 * @param err 电机状态错误值
 */
static void motor_driver_update_state_err(motor_driver_t *self, uint16_t err)
{
    if ((self->info->state == MOTOR_STATE_ALARM) ||
        (self->info->state == JOINT_ALARM))
    {
        self->info->state_err = (motor_err_t)err;
    }
}

/**
 * @brief 重置电机
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_reset_motor(motor_driver_t *self)
{
    if(self->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        SDO_Write_Data2(self->info->id, 0x4603, 0x00, 1);
    }
    else if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        SDO_Write_Data2(self->info->id, 0x6040, 0x00, 0x0086);
    }
}

/**
 * @brief 重置电机零位
 * 
 * @param self 电机驱动程序结构体指针
 */
static void motor_driver_reset_zero(motor_driver_t *self)
{
    if(self->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS)
    {
        SDO_Write_Data2(self->info->id, 0x2004, 0x00, 0x0001);
    }
}

/**
 * @brief 创建电机驱动程序
 * 
 * @param driver 电机驱动程序结构体指针
 * @param info 电机信息结构体指针
 */
void motor_driver_create(motor_driver_t *driver, motor_info_t *info) 
{
    driver->info = info;

    driver->init = __motor_driver_real_init;
    driver->enter_ready = motor_driver_enter_ready;
    driver->enable = motor_driver_enable;
    driver->disable = motor_driver_disable;
    driver->set_speed = motor_driver_set_speed;
    driver->set_relative_position = motor_driver_set_relative_position;
    driver->set_absolute_position = motor_driver_set_absolute_position;

    driver->inquire_speed = motor_driver_inquire_speed;
    driver->inquire_encoder = motor_driver_inquire_encoder;
    driver->inquire_state = motor_driver_inquire_state;
    driver->inquire_state_err = motor_driver_inquire_state_err;

    driver->update_speed = motor_driver_update_speed;
    driver->update_encoder = motor_driver_update_encoder;
    driver->update_status = motor_driver_update_state;
    driver->update_state_err = motor_driver_update_state_err;

    driver->reset_motor = motor_driver_reset_motor;
    driver->reset_zero = motor_driver_reset_zero;
}
