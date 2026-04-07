#include "bsp_gpio_driver.h"
#include "gpio_port.h"

#include "system_cfg.h"

#ifndef DEBUG
	static input_status_t input_status = {0};
#else
	input_status_t input_status = {0};
#endif

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
