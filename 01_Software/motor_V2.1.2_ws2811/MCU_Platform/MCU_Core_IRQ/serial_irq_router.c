#include "serial_irq_router.h"

#include <stdint.h>

#include "usart.h"

#include "bsp_display_driver.h"
#include "bsp_battery_driver.h"
#include "bsp_ibus_driver.h"


void mcu_usart2_irq_route(void)
{
    uint8_t data = 0u;

    if (LL_USART_IsActiveFlag_RXNE(USART2))
    {
        data = (uint8_t)USART2->DR;
        display_driver_input_byte(data);
    }
}

void mcu_usart3_irq_route(void)
{
    uint8_t data = 0u;

    if (LL_USART_IsActiveFlag_RXNE(USART3))
    {
        data = (uint8_t)USART3->DR;
        modbus_driver_input_byte(data);
    }
}

void mcu_uart4_irq_route(void)
{
    uint8_t data = 0u;

    if (LL_USART_IsActiveFlag_RXNE(UART4))
    {
        data = (uint8_t)UART4->DR;
        ibus_driver_input_byte(data);
    }
}
