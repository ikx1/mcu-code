/**
 * @file bsp_can_handler.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _BSP_MOTOR_HANDLER_H_
#define _BSP_MOTOR_HANDLER_H_

/********************************** Includes *********************************/
#include <stdint.h>

#include "bsp_motor_driver.h"

/********************************** Defines **********************************/
#define MOTOR_NUM 3

typedef enum
{
    MOTOR_INDEX_RIGHT = 0,
    MOTOR_INDEX_LEFT = 1,
    MOTOR_INDEX_JOINT = 2,
} motor_index_t;


/********************************** Variables ********************************/
typedef struct
{
    motor_state_t state;
    uint8_t initialized;
    uint8_t ready_flag;
} motor_runtime_t;


/********************************** Functions ********************************/
uint8_t motor_info_snapshot(motor_index_t index, motor_info_t *out);
uint8_t motor_runtime_snapshot(motor_index_t index, motor_runtime_t *out);
uint8_t motor_runtime_set_ready_flag(motor_index_t index, uint8_t ready_flag);
uint8_t motor_runtime_reset_init_state(motor_index_t index);

uint8_t motor_cmd_init(motor_index_t index);
uint8_t motor_cmd_enter_ready(motor_index_t index);
uint8_t motor_cmd_enable(motor_index_t index);
uint8_t motor_cmd_disable(motor_index_t index);
uint8_t motor_cmd_set_speed(motor_index_t index, int32_t rpm);
uint8_t motor_cmd_set_relative_position(motor_index_t index, int32_t pos);
uint8_t motor_cmd_set_absolute_position(motor_index_t index, int32_t pos);
uint8_t motor_cmd_inquire_encoder(motor_index_t index);
uint8_t motor_cmd_inquire_state(motor_index_t index);
uint8_t motor_cmd_inquire_state_err(motor_index_t index);
uint8_t motor_cmd_reset_motor(motor_index_t index);
uint8_t motor_cmd_reset_zero(motor_index_t index);

void motor_register(void);

void ican_rec_analyse(uint32_t std_id, const uint8_t *rx_data, uint8_t dlc);
void can_poll_parse_loop(void);

#endif /* _BSP_CAN_HANDLER_H_ */
