#ifndef GPIO_PORT_H
#define GPIO_PORT_H

#include <stdint.h>

typedef struct
{
    uint8_t emergency_flag;
    uint8_t limit_upper;
    uint8_t limit_lower;
    uint8_t limit_c;
} gpio_port_input_state_t;

void gpio_port_read_inputs(gpio_port_input_state_t *state);

#endif /* GPIO_PORT_H */
