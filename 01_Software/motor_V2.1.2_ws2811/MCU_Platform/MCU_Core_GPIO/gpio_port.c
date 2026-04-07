#include "gpio_port.h"

#include "main.h"

void gpio_port_read_inputs(gpio_port_input_state_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->emergency_flag = (uint8_t)(HAL_GPIO_ReadPin(IO1_GPIO_Port, IO1_Pin) == GPIO_PIN_SET);
    state->limit_upper = (uint8_t)(HAL_GPIO_ReadPin(IO2_GPIO_Port, IO2_Pin) == GPIO_PIN_RESET);
    state->limit_lower = (uint8_t)(HAL_GPIO_ReadPin(IO3_GPIO_Port, IO3_Pin) == GPIO_PIN_RESET);
    state->limit_c = (uint8_t)(HAL_GPIO_ReadPin(IO4_GPIO_Port, IO4_Pin) == GPIO_PIN_SET);
}
