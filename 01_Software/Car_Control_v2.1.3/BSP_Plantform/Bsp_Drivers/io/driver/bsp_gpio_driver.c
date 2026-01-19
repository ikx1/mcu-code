/**
 * @file bsp_gpio_driver.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_gpio_driver.h"
#include "main.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef struct 
{
	GPIO_TypeDef* port;
	uint16_t pin;
	uint8_t active_low;
	uint8_t* target_flag;
}gpio_input_cfg_t;

static input_status_t input_status = {0};

static const gpio_input_cfg_t gpio_inputs[] = {
	{ IO1_GPIO_Port, IO1_Pin, 0, &input_status.limit_c }, // 高电平有效
	{ IO2_GPIO_Port, IO2_Pin, 1, &input_status.limit_a },        // 低电平有效
	{ IO3_GPIO_Port, IO3_Pin, 1, &input_status.limit_b },        // 低电平有效
	{ IO4_GPIO_Port, IO4_Pin, 0, &input_status.emergency_flag },        // 低电平有效
};

/********************************** Functions ********************************/
void input_driver_update_10ms(void)
{
	for (int i = 0; i < sizeof(gpio_inputs)/sizeof(gpio_inputs[0]); ++i)
	{
		uint8_t raw = HAL_GPIO_ReadPin(gpio_inputs[i].port, gpio_inputs[i].pin);
		*(gpio_inputs[i].target_flag) = gpio_inputs[i].active_low ? !raw : raw;
	}
}

const input_status_t* input_driver_get_status(void)
{
	return &input_status;
}
