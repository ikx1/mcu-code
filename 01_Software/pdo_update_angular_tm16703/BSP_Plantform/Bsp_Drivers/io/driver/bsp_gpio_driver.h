#ifndef BSP_GPIO_DRIVER_H
#define BSP_GPIO_DRIVER_H

#include <stdint.h>

typedef struct
{
    uint8_t emergency_flag;
    uint8_t limit_upper;
    uint8_t limit_lower;
    uint8_t limit_c;
} input_status_t;

void input_driver_update_10ms(void);
const input_status_t *input_driver_get_status(void);
void relay1_gpio_set(uint8_t enable);
void relay2_gpio_set(uint8_t enable);
void relay_gpio_con(uint8_t push_mode);

#endif /* BSP_GPIO_DRIVER_H */
