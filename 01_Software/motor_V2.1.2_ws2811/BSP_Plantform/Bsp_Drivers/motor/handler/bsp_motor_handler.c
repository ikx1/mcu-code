/**
 * @file bsp_can_handler.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_motor_handler.h"
#include "can_queue.h"
#include "irq_guard.h"


/********************************** Defines **********************************/
#define SERVO_SDO_RESP_BASE_ID 0x580u
#define SERVO_TPDO1_BASE_ID    0x180u

typedef struct
{
    uint32_t node_id;
    uint32_t sdo_resp_id;
    uint32_t tpdo1_id;
    servo_pdo_layout_t pdo_layout;
} motor_slot_cfg_t;


/********************************** Variables ********************************/
static const motor_slot_cfg_t motor_slot_cfg[MOTOR_NUM] = {
    [MOTOR_INDEX_RIGHT] = {
        .node_id = Right_Wheel_ID,
        .sdo_resp_id = SERVO_SDO_RESP_BASE_ID + Right_Wheel_ID,
        .tpdo1_id = SERVO_TPDO1_BASE_ID + Right_Wheel_ID,
        .pdo_layout = SERVO_SDO_LAYOUT_DRIVE_VEL,
    },
    [MOTOR_INDEX_LEFT] = {
        .node_id = Left_Wheel_ID,
        .sdo_resp_id = SERVO_SDO_RESP_BASE_ID + Left_Wheel_ID,
        .tpdo1_id = SERVO_TPDO1_BASE_ID + Left_Wheel_ID,
        .pdo_layout = SERVO_SDO_LAYOUT_DRIVE_VEL,
    },
    [MOTOR_INDEX_JOINT] = {
        .node_id = Joint_Wheel_ID,
        .sdo_resp_id = SERVO_SDO_RESP_BASE_ID + Joint_Wheel_ID,
        .tpdo1_id = SERVO_TPDO1_BASE_ID + Joint_Wheel_ID,
        .pdo_layout = SERVO_PDO_LAYOUT_SLIDE_POS,
    },
};

#ifndef DEBUG
    static motor_info_t motor_infos[MOTOR_NUM];
    static motor_driver_t motor_drivers[MOTOR_NUM];
#else
    motor_info_t motor_infos[MOTOR_NUM];
    motor_driver_t motor_drivers[MOTOR_NUM];
#endif

/********************************** Functions ********************************/
/**
 * @brief 保存电机中断屏蔽标志
 * 
 * @return uint32_t 中断屏蔽标志
 */
static uint32_t motor_irq_save(void)
{
    return mcu_irq_guard_lock();
}

/**
 * @brief 恢复电机中断屏蔽标志
 * 
 * @param primask 中断屏蔽标志
 */
static void motor_irq_restore(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

/**
 * @brief 获取电机驱动程序
 * 
 * @param index 电机索引
 * @return motor_driver_t* 电机驱动程序结构体指针
 */
static motor_driver_t* motor_get_driver(motor_index_t index)
{
    if ((uint32_t)index >= (uint32_t)MOTOR_NUM)
    {
        return NULL;
    }

    return &motor_drivers[(uint32_t)index];
}

/**
 * @brief 获取电机信息快照
 * 
 * @param index 电机索引
 * @param out 电机信息结构体指针
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_info_snapshot(motor_index_t index, motor_info_t *out)
{
    uint32_t primask;
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->info == NULL) || (out == NULL))
    {
        return 1u;
    }

    primask = motor_irq_save();
    *out = *(drv->info);
    motor_irq_restore(primask);
    return 0u;
}

/**
 * @brief 获取电机运行时快照
 * 
 * @param index 电机索引
 * @param out 电机运行时结构体指针
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_runtime_snapshot(motor_index_t index, motor_runtime_t *out)
{
    uint32_t primask;
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->info == NULL) || (out == NULL))
    {
        return 1u;
    }

    primask = motor_irq_save();
    out->state = drv->info->state;
    out->initialized = drv->init_state.initialized;
    out->ready_flag = drv->init_state.ready_flag;
    motor_irq_restore(primask);
    return 0u;
}

/**
 * @brief 设置电机运行时准备标志
 * 
 * @param index 电机索引
 * @param ready_flag 准备标志
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_runtime_set_ready_flag(motor_index_t index, uint8_t ready_flag)
{
    uint32_t primask;
    motor_driver_t *drv = motor_get_driver(index);
    if (drv == NULL)
    {
        return 1u;
    }

    primask = motor_irq_save();
    drv->init_state.ready_flag = ready_flag;
    motor_irq_restore(primask);
    return 0u;
}

/**
 * @brief 重置电机初始化状态
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_runtime_reset_init_state(motor_index_t index)
{
    uint32_t primask;
    motor_driver_t *drv = motor_get_driver(index);
    if (drv == NULL)
    {
        return 1u;
    }

    primask = motor_irq_save();
    drv->init_state.initialized = NOT_INITED;
    drv->init_state.init_type = MOTOR_INIT_NMT_PREOP;
    drv->init_state.ready_flag = 0u;
    motor_irq_restore(primask);
    return 0u;
}

/**
 * @brief 初始化电机
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_init(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->init == NULL))
    {
        return 1u;
    }

    drv->init(drv);
    return 0u;
}

/**
 * @brief 进入准备状态
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_enter_ready(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->enter_ready == NULL))
    {
        return 1u;
    }

    drv->enter_ready(drv);
    return 0u;
}

/**
 * @brief 使能电机
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_enable(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->enable == NULL))
    {
        return 1u;
    }

    drv->enable(drv);
    return 0u;
}

/**
 * @brief 禁用电机
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_disable(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->disable == NULL))
    {
        return 1u;
    }

    drv->disable(drv);
    return 0u;
}

/**
 * @brief 设置电机速度
 * 
 * @param index 电机索引
 * @param rpm 速度值（单位：转/分钟）
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_set_speed(motor_index_t index, int32_t rpm)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->set_speed == NULL))
    {
        return 1u;
    }

    drv->set_speed(drv, rpm);
    return 0u;
}

/**
 * @brief 设置电机相对位置
 * 
 * @param index 电机索引
 * @param pos 相对位置值（单位：度）
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_set_relative_position(motor_index_t index, int32_t pos)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->set_relative_position == NULL))
    {
        return 1u;
    }

    drv->set_relative_position(drv, pos);
    return 0u;
}

/**
 * @brief 设置电机绝对位置
 * 
 * @param index 电机索引
 * @param pos 绝对位置值（单位：度）
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_set_absolute_position(motor_index_t index, int32_t pos)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->set_absolute_position == NULL))
    {
        return 1u;
    }

    drv->set_absolute_position(drv, pos);
    return 0u;
}

/**
 * @brief 重置电机
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_inquire_encoder(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->inquire_encoder == NULL))
    {
        return 1u;
    }

    drv->inquire_encoder(drv);
    return 0u;
}

uint8_t motor_cmd_inquire_state(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->inquire_state == NULL))
    {
        return 1u;
    }

    drv->inquire_state(drv);
    return 0u;
}

uint8_t motor_cmd_inquire_state_err(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->inquire_state_err == NULL))
    {
        return 1u;
    }

    drv->inquire_state_err(drv);
    return 0u;
}

uint8_t motor_cmd_reset_motor(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->reset_motor == NULL))
    {
        return 1u;
    }

    drv->reset_motor(drv);
    return 0u;
}

/**
 * @brief 重置电机零位
 * 
 * @param index 电机索引
 * @return uint8_t 0 成功 1 失败
 */
uint8_t motor_cmd_reset_zero(motor_index_t index)
{
    motor_driver_t *drv = motor_get_driver(index);
    if ((drv == NULL) || (drv->reset_zero == NULL))
    {
        return 1u;
    }

    drv->reset_zero(drv);
    return 0u;
}

/**
 * @brief 从字节数组中提取小端16位整数
 * 
 * @param d 字节数组指针
 * @param off 偏移量
 * @return uint16_t 小端16位整数
 */
static inline uint16_t servo_le16(const uint8_t d[8], uint8_t off)
{
	return (uint16_t)((uint16_t)d[off] | ((uint16_t)d[off + 1u] << 8));
}

static inline int16_t servo_le16s(const uint8_t d[8], uint8_t off)
{
	return (int16_t)servo_le16(d, off);
}

static inline int32_t servo_le32s(const uint8_t d[8], uint8_t off)
{
	return (int32_t)((uint32_t)d[off] |
			 ((uint32_t)d[off + 1u] << 8) |
			 ((uint32_t)d[off + 2u] << 16) |
			 ((uint32_t)d[off + 3u] << 24));
}

static motor_driver_t* motor_find_driver_by_std_id(uint32_t std_id)
{
    for (uint32_t i = 0; i < (uint32_t)MOTOR_NUM; ++i)
    {
        if (motor_slot_cfg[i].tpdo1_id == std_id)
        {
            return &motor_drivers[i];
        }

        if ((motor_slot_cfg[i].sdo_resp_id != 0u) &&
            (motor_slot_cfg[i].sdo_resp_id == std_id))
        {
            return &motor_drivers[i];
        }
    }

    return NULL;
}

static void motor_parse_drive_sdo_response(motor_driver_t *target_motor,
                                           const uint8_t rx_data[8])
{
    uint8_t cmd = rx_data[0];
    uint16_t index = servo_le16(rx_data, 1u);

    switch (cmd)
    {
        case 0x43u:
            if (index == 0x6069u)
            {
                int32_t speed = servo_le32s(rx_data, 4u);
                target_motor->update_speed(target_motor, speed);
            }
            else if (index == 0x6063u)
            {
                int32_t pos = servo_le32s(rx_data, 4u);
                target_motor->update_encoder(target_motor, pos);
            }
            else if (index == 0x603Fu)
            {
                uint16_t state_err = servo_le16(rx_data, 4u);
                target_motor->update_state_err(target_motor, state_err);
            }
            break;

        case 0x4Bu:
            if (index == 0x6041u)
            {
                uint16_t status = servo_le16(rx_data, 4u);
                target_motor->update_status(target_motor, status);
            }
            else if (index == 0x603Fu)
            {
                uint16_t state_err = servo_le16(rx_data, 4u);
                target_motor->update_state_err(target_motor, state_err);
            }
            break;

        default:
            break;
    }
}

static void motor_parse_drive_tpdo1(motor_driver_t *target_motor,
                                    const uint8_t rx_data[8])
{
    uint16_t status = servo_le16(rx_data, 0u);
    int32_t speed = servo_le32s(rx_data, 2u);

    target_motor->update_status(target_motor, status);
    target_motor->update_speed(target_motor, speed);
}

static void motor_parse_joint_tpdo1(motor_driver_t *target_motor,
                                    const uint8_t rx_data[8])
{
    uint16_t status = servo_le16(rx_data, 0u);
    int16_t vel_raw16 = servo_le16s(rx_data, 2u);
    int32_t pos_raw = servo_le32s(rx_data, 4u);

    target_motor->update_status(target_motor, status);
    target_motor->update_speed(target_motor, vel_raw16);
    target_motor->update_encoder(target_motor, pos_raw);
}

static void motor_parse_joint_sdo_response(motor_driver_t *target_motor,
                                           const uint8_t rx_data[8])
{
    uint8_t cmd = rx_data[0];
    uint16_t index = servo_le16(rx_data, 1u);

    switch (cmd)
    {
        case 0x43u:
            if (index == 0x6063u)
            {
                int32_t pos = servo_le32s(rx_data, 4u);
                target_motor->update_encoder(target_motor, pos);
            }
            else if (index == 0x6069u)
            {
                int32_t speed = servo_le32s(rx_data, 4u);
                target_motor->update_speed(target_motor, speed);
            }
            else if (index == 0x603Fu)
            {
                uint16_t state_err = servo_le16(rx_data, 4u);
                target_motor->update_state_err(target_motor, state_err);
            }
            break;

        case 0x4Bu:
            if (index == 0x6041u)
            {
                uint16_t status = servo_le16(rx_data, 4u);
                target_motor->update_status(target_motor, status);
            }
            else if (index == 0x603Fu)
            {
                uint16_t state_err = servo_le16(rx_data, 4u);
                target_motor->update_state_err(target_motor, state_err);
            }
            break;

        default:
            break;
    }
}

static void motor_parse_frame(motor_driver_t *target_motor,
                              const uint8_t rx_data[8],
                              uint8_t dlc)
{
    if ((target_motor == NULL) || (target_motor->info == NULL))
    {
        return;
    }

    if (target_motor->pdo_layout == SERVO_SDO_LAYOUT_DRIVE_VEL)
    {
        if (dlc == 8u)
        {
            motor_parse_drive_sdo_response(target_motor, rx_data);
        }
        else if (dlc == 6u)
        {
            motor_parse_drive_tpdo1(target_motor, rx_data);
        }

        return;
    }

    if ((target_motor->pdo_layout == SERVO_PDO_LAYOUT_SLIDE_POS) && (dlc == 8u))
    {
        if ((rx_data[0] == 0x43u) || (rx_data[0] == 0x4Bu))
        {
            motor_parse_joint_sdo_response(target_motor, rx_data);
        }
        else
        {
            motor_parse_joint_tpdo1(target_motor, rx_data);
        }
    }
}

void motor_register(void)
{
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        motor_infos[i].id = motor_slot_cfg[i].node_id;
        motor_infos[i].state = MOTOR_STATE_UNKNOWN;
        motor_infos[i].state_err = ERR_OK;
        motor_infos[i].current_speed = 0;
        motor_infos[i].wheel_speed = 0.0f;
        motor_infos[i].wheel_linear = 0.0f;
        motor_infos[i].wheel_angular = 0.0f;
        motor_infos[i].set_speed = 0;
        motor_infos[i].current_position = 0;
        motor_infos[i].location = 0.0f;
        motor_infos[i].last_position = 0;
        motor_infos[i].set_position = 0;
        motor_infos[i].initialized = NOT_INITED;

        motor_drivers[i].pdo_layout = motor_slot_cfg[i].pdo_layout;
        motor_drivers[i].init_state.init_type = MOTOR_INIT_NMT_PREOP;
        motor_drivers[i].init_state.initialized = NOT_INITED;
        motor_drivers[i].init_state.ready_flag = 0u;

        motor_driver_create(&motor_drivers[i], &motor_infos[i]);
    }
}

/* Parse one standard CAN frame. */
void ican_rec_analyse(uint32_t std_id, const uint8_t *rx_data, uint8_t dlc)
{
    motor_driver_t *target_motor = NULL;

    if (rx_data == NULL)
    {
        return;
    }

    target_motor = motor_find_driver_by_std_id(std_id);
    if (target_motor == NULL)
    {
        return;
    }

    motor_parse_frame(target_motor, rx_data, dlc);
}

void can_poll_parse_loop(void)
{
    can_std_frame_t frame;

    while (can_rx_fifo_pop(&frame))
    {
        ican_rec_analyse(frame.std_id, frame.data, frame.dlc);
    }
}
