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
#include "can_queue.h"

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
static volatile uint8_t motor_cfg_status = 0;

/********************************** Functions ********************************/

/********************************** PDO Disable/Clear **********************************/
static void motor_pdo_disable_and_clear(uint8_t id)
{
    /* 1) 先禁用 PDO（COB-ID bit31=1），同时写入你期望的 COB-ID（更稳） */
    // SDO_Write_Data4(id, SERVO_IDX_RPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_RPDO1(id));
    // SDO_Write_Data4(id, SERVO_IDX_RPDO2_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_RPDO2(id));
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO1(id));
    SDO_Write_Data4(id, SERVO_IDX_TPDO2_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO2(id));

    /* 2) 清映射：sub0=0（标准 remap 第一步） */
    // SDO_Write_Data1(id, SERVO_IDX_RPDO1_MAP, 0x00, 0x00);
    // SDO_Write_Data1(id, SERVO_IDX_RPDO2_MAP, 0x00, 0x00);
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x00);
    SDO_Write_Data1(id, SERVO_IDX_TPDO2_MAP, 0x00, 0x00);
}

/********************************** PDO Mapping (标准顺序) **********************************/
//static void motor_cfg_rpdo1(uint8_t id)
//{
//    /* A) 禁用 */
//    SDO_Write_Data4(id, SERVO_IDX_RPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_RPDO1(id));

//    /* B) 通信参数（可选，但建议写清楚） */
//    SDO_Write_Data1(id, SERVO_IDX_RPDO1_COMM, 0x02, 0xFE);   // transmission type（你原来注释掉的）

//    /* C) remap：先0 */
//    SDO_Write_Data1(id, SERVO_IDX_RPDO1_MAP, 0x00, 0x00);

//    /* D) 写映射项 */
//    // SDO_Write_Data4(id, SERVO_IDX_RPDO1_MAP, 0x01, SERVO_RPDO_MAP_CONTROL);
//    SDO_Write_Data4(id, SERVO_IDX_RPDO1_MAP, 0x01, SERVO_RPDO_MAP_TARGET_SPEED);

//    /* E) 写映射数量 */
//    SDO_Write_Data1(id, SERVO_IDX_RPDO1_MAP, 0x00, 0x01);

//    /* F) 启用 */
//    SDO_Write_Data4(id, SERVO_IDX_RPDO1_COMM, 0x01, SERVO_COB_RPDO1(id));
//}

// static void motor_cfg_rpdo2(uint8_t id)
// {
//     /* A) 禁用 */
//     SDO_Write_Data4(id, SERVO_IDX_RPDO2_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_RPDO2(id));

//     /* B) 通信参数（可选） */
//     SDO_Write_Data1(id, SERVO_IDX_RPDO2_COMM, 0x02, 0xFE);

//     /* C) remap：先0 */
//     SDO_Write_Data1(id, SERVO_IDX_RPDO2_MAP, 0x00, 0x00);

//     /* D) 写映射项（目标位置 0x607A:00 32bit） */
//     SDO_Write_Data4(id, SERVO_IDX_RPDO2_MAP, 0x01, SERVO_RPDO_MAP_TARGET_POS);

//     /* E) 写映射数量 */
//     SDO_Write_Data1(id, SERVO_IDX_RPDO2_MAP, 0x00, 0x01);

//     /* F) 启用 */
//     SDO_Write_Data4(id, SERVO_IDX_RPDO2_COMM, 0x01, SERVO_COB_RPDO2(id));
// }

static void motor_cfg_tpdo1(uint8_t id)
{
    /* A) 禁用（你原来就做了，这里保留并规范成“写期望COB-ID|disable”） */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, PDO_DISABLE_MASK | SERVO_COB_TPDO1(id));

    /* B) 通信参数 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_COMM, 0x02, 0xFE);   // transmission type
    SDO_Write_Data2(id, SERVO_IDX_TPDO1_COMM, 0x03, 0x0032); // inhibit time
    SDO_Write_Data2(id, SERVO_IDX_TPDO1_COMM, 0x05, 0x0032); // event timer

    /* C) remap：先0 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x00);

    /* D) 写映射项 */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x01, SERVO_TPDO_MAP_STATUS);
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x02, SERVO_TPDO_MAP_ACTUAL_POS);
    // SDO_Write_Data4(id, SERVO_IDX_TPDO1_MAP, 0x02, SERVO_TPDO_MAP_ACTUAL_SPEED);

    /* E) 写映射数量 */
    SDO_Write_Data1(id, SERVO_IDX_TPDO1_MAP, 0x00, 0x02);

    /* F) 启用 */
    SDO_Write_Data4(id, SERVO_IDX_TPDO1_COMM, 0x01, SERVO_COB_TPDO1(id));
}

static void __motor_driver_real_init(motor_driver_t *self)
{
    if (self->init_state.initialized == INITED) return;

    uint8_t id = self->info->id;

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

static void motor_driver_enter_ready(motor_driver_t *self)
{
    SDO_Write_Data2(self->info->id, 0x6040, 0x00, 0x06);
}

static void motor_driver_enable(motor_driver_t *self) 
{
    SDO_Write_Data2(self->info->id, 0x6040, 0x00, 0x0f);
}

static void motor_driver_disable(motor_driver_t *self) 
{
    SDO_Write_Data2(self->info->id, 0x6040, 0x00, 0x07);
}

static void motor_driver_set_speed(motor_driver_t *self, int32_t rpm) 
{
    self->info->set_speed = rpm;
    SDO_Write_Data4(self->info->id, 0x60ff, 0x00, (int32_t)rpm);
    // RPDO1_Write_Cmd_Data4(self->info->id, rpm);
}

static void motor_driver_set_encoder(motor_driver_t *self, int32_t pos) 
{
    self->info->set_position = pos;
    RPDO2_Write_Cmd_Data4(self->info->id, pos);
}

static void motor_driver_inquire_speed(uint8_t self)
{
    SDO_Read_Data4(self, 0x6069, 0x00);
}

static void motor_driver_inquire_encoder(uint8_t self)
{
    SDO_Read_Data4(self, 0x6063, 0x00);
}

static void motor_driver_inquire_state(uint8_t self)
{
    SDO_Read_Data4(self, 0x6041, 0x00);
}

static void motor_driver_update_speed(motor_driver_t *self, int32_t rpm)
{
    self->info->current_speed = rpm;
    self->info->wheel_linear = (float)rpm * WHEEL_PERIMETER / 60.0;
}
static void motor_driver_update_encoder(motor_driver_t *self, int32_t val)
{
    self->info->last_position = self->info->current_position;
    self->info->current_position = val;
    self->info->wheel_speed = (self->info->current_position - 
                                                     self->info->last_position) 
                                               * 20 / 5600.0 * WHEEL_PERIMETER;
}
static void motor_driver_update_state(motor_driver_t *self, uint16_t state)
{
    self->info->state = (motor_state_t)state;
}

static void motor_driver_update_state_err(motor_driver_t *self, uint16_t err)
{
    self->info->state_err = (motor_err_t)err;
}

static void motor_driver_reset_motor(uint8_t self)
{
    SDO_Write_Data2(self, 0x4603, 0x00, 1);
}


void motor_driver_create(motor_driver_t *driver, motor_info_t *info) 
{
    driver->info = info;

    driver->init = __motor_driver_real_init;
    driver->enter_ready = motor_driver_enter_ready;
    driver->enable = motor_driver_enable;
    driver->disable = motor_driver_disable;
    driver->set_speed = motor_driver_set_speed;
    driver->set_position = motor_driver_set_encoder;

    driver->inquire_speed = motor_driver_inquire_speed;
    driver->inquire_encoder = motor_driver_inquire_encoder;
    driver->inquire_state = motor_driver_inquire_state;
    
    driver->update_speed = motor_driver_update_speed;
    driver->update_encoder = motor_driver_update_encoder;
    driver->update_state = motor_driver_update_state;
    driver->update_state_err = motor_driver_update_state_err;

    driver->reset_motor = motor_driver_reset_motor;
}
