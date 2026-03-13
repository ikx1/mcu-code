#include "bsp_gpio_driver.h"
#include "gpio_port.h"

#include "system_cfg.h"
#include "stm32f10x_gpio.h"

#ifndef DEBUG
	static input_status_t input_status = {0};
#else
	input_status_t input_status = {0};
#endif

#define BSP_GPIO_RELAY1_PORT    GPIOB
#define BSP_GPIO_RELAY1_PIN     GPIO_Pin_0
#define BSP_GPIO_RELAY2_PORT    GPIOB
#define BSP_GPIO_RELAY2_PIN     GPIO_Pin_1

/**
 * @brief 更新输入状态
 * 
 */
void input_driver_update_10ms(void)
{
    gpio_port_input_state_t raw_state;

    gpio_port_read_inputs(&raw_state);

    input_status.emergency_flag = raw_state.emergency_flag;
    input_status.limit_upper = raw_state.limit_upper;
    input_status.limit_lower = raw_state.limit_lower;
    input_status.limit_c = raw_state.limit_c;
}

/**
 * @brief 获取输入状态
 * 
 * @return const input_status_t* 输入状态结构体指针
 */
const input_status_t *input_driver_get_status(void)
{
    return &input_status;
}

static void bsp_gpio_set_output(GPIO_TypeDef *port, uint16_t pin, uint8_t enable)
{
    if (enable != 0u)
    {
        GPIO_SetBits(port, pin);
    }
    else
    {
        GPIO_ResetBits(port, pin);
    }
}

void relay1_gpio_set(uint8_t enable)
{
    bsp_gpio_set_output(BSP_GPIO_RELAY1_PORT, BSP_GPIO_RELAY1_PIN, enable);
}

void relay2_gpio_set(uint8_t enable)
{
    bsp_gpio_set_output(BSP_GPIO_RELAY2_PORT, BSP_GPIO_RELAY2_PIN, enable);
}

void relay_gpio_con(uint8_t push_mode)
{
    /* Keep the legacy single-relay API mapped to relay2, which is PB1 on the
     * current board and matches the old demo behavior. */
    relay2_gpio_set(push_mode);
}
