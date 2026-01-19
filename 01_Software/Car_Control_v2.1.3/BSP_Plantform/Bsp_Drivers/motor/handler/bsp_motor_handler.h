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
#include "bsp_motor_driver.h"
#include "stm32f4xx.h"

/********************************** Defines **********************************/
#define MOTOR_NUM 2


/********************************** Variables ********************************/

/********************************** Functions ********************************/
motor_driver_t* get_motor_drivers_p(void);

void motor_register(void);

void ican_rec_analyse(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, uint8_t DLC);
void can_poll_parse_loop(void);

#endif /* _BSP_CAN_HANDLER_H_ */
