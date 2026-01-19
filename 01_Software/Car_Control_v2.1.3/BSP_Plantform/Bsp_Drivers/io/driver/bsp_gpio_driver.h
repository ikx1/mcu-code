/**
 * @file bsp_gpio_driver.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_GPIO_DRIVER_H__
#define __BSP_GPIO_DRIVER_H__

/********************************** Includes *********************************/
#include "gpio.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef struct 
{
	uint8_t emergency_flag;  // 急停：1-按下，0-未按
	uint8_t limit_a;         // 限位A：1-触发，0-未触发
	uint8_t limit_b;         // 限位B：1-触发，0-未触发
	uint8_t limit_c;
} input_status_t;

/********************************** Functions ********************************/
/**
 * @brief 每 10ms 调用一次，更新输入状态
 */
void input_driver_update_10ms(void);

/**
 * @brief 获取当前输入状态结构体指针
 */
const input_status_t* input_driver_get_status(void);
void relay_gpio_con(uint8_t push_mode);


#endif /* __BSP_GPIO_DRIVER_H__ */
