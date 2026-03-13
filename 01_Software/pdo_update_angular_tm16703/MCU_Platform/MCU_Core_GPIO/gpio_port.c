#include "gpio_port.h"

#include "stm32f10x_gpio.h"

/*
 * Default board mapping follows the pins initialized in Core/Src/gpio.c:
 * PE7  : emergency input, active high to match the stable F103 demo board
 * PE8  : upper limit, active low
 * PE9  : lower limit, active low
 * limit_c is left disabled by default because the current board init does not
 * configure a fourth dedicated input pin.
 */
#ifndef GPIO_PORT_EMERGENCY_GPIO_PORT
#define GPIO_PORT_EMERGENCY_GPIO_PORT GPIOE
#endif

#ifndef GPIO_PORT_EMERGENCY_GPIO_PIN
#define GPIO_PORT_EMERGENCY_GPIO_PIN  GPIO_Pin_7
#endif

#ifndef GPIO_PORT_EMERGENCY_ACTIVE_LEVEL
#define GPIO_PORT_EMERGENCY_ACTIVE_LEVEL Bit_SET
#endif

#ifndef GPIO_PORT_LIMIT_UPPER_GPIO_PORT
#define GPIO_PORT_LIMIT_UPPER_GPIO_PORT GPIOE
#endif

#ifndef GPIO_PORT_LIMIT_UPPER_GPIO_PIN
#define GPIO_PORT_LIMIT_UPPER_GPIO_PIN  GPIO_Pin_8
#endif

#ifndef GPIO_PORT_LIMIT_UPPER_ACTIVE_LEVEL
#define GPIO_PORT_LIMIT_UPPER_ACTIVE_LEVEL Bit_RESET
#endif

#ifndef GPIO_PORT_LIMIT_LOWER_GPIO_PORT
#define GPIO_PORT_LIMIT_LOWER_GPIO_PORT GPIOE
#endif

#ifndef GPIO_PORT_LIMIT_LOWER_GPIO_PIN
#define GPIO_PORT_LIMIT_LOWER_GPIO_PIN  GPIO_Pin_9
#endif

#ifndef GPIO_PORT_LIMIT_LOWER_ACTIVE_LEVEL
#define GPIO_PORT_LIMIT_LOWER_ACTIVE_LEVEL Bit_RESET
#endif

#ifndef GPIO_PORT_LIMIT_C_PRESENT
#define GPIO_PORT_LIMIT_C_PRESENT 0u
#endif

#if (GPIO_PORT_LIMIT_C_PRESENT != 0u)
#ifndef GPIO_PORT_LIMIT_C_GPIO_PORT
#define GPIO_PORT_LIMIT_C_GPIO_PORT GPIOE
#endif

#ifndef GPIO_PORT_LIMIT_C_GPIO_PIN
#define GPIO_PORT_LIMIT_C_GPIO_PIN  GPIO_Pin_10
#endif

#ifndef GPIO_PORT_LIMIT_C_ACTIVE_LEVEL
#define GPIO_PORT_LIMIT_C_ACTIVE_LEVEL Bit_SET
#endif
#endif

static uint8_t gpio_port_read_level(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, BitAction active_level)
{
    return (GPIO_ReadInputDataBit(gpio_port, gpio_pin) == active_level) ? 1u : 0u;
}

void gpio_port_read_inputs(gpio_port_input_state_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->emergency_flag = gpio_port_read_level(GPIO_PORT_EMERGENCY_GPIO_PORT,
                                                 GPIO_PORT_EMERGENCY_GPIO_PIN,
                                                 GPIO_PORT_EMERGENCY_ACTIVE_LEVEL);
    state->limit_upper = gpio_port_read_level(GPIO_PORT_LIMIT_UPPER_GPIO_PORT,
                                              GPIO_PORT_LIMIT_UPPER_GPIO_PIN,
                                              GPIO_PORT_LIMIT_UPPER_ACTIVE_LEVEL);
    state->limit_lower = gpio_port_read_level(GPIO_PORT_LIMIT_LOWER_GPIO_PORT,
                                              GPIO_PORT_LIMIT_LOWER_GPIO_PIN,
                                              GPIO_PORT_LIMIT_LOWER_ACTIVE_LEVEL);
#if (GPIO_PORT_LIMIT_C_PRESENT != 0u)
    state->limit_c = gpio_port_read_level(GPIO_PORT_LIMIT_C_GPIO_PORT,
                                          GPIO_PORT_LIMIT_C_GPIO_PIN,
                                          GPIO_PORT_LIMIT_C_ACTIVE_LEVEL);
#else
    state->limit_c = 0u;
#endif
}
