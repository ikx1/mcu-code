#include "serial_irq_router.h"

#include <stddef.h>
#include <stdint.h>

#include "legacy_uart_port.h"

#include "stm32f10x_usart.h"

static void serial_irq_route_byte(USART_TypeDef *uart, legacy_uart_rx_byte_cb_t rx_isr)
{
    if (USART_GetFlagStatus(uart, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(uart);
    }

    if ((USART_GetITStatus(uart, USART_IT_RXNE) != RESET) && (rx_isr != NULL))
    {
        rx_isr((uint8_t)USART_ReceiveData(uart));
    }

    if (USART_GetFlagStatus(uart, USART_FLAG_IDLE) != RESET)
    {
        (void)USART_ReceiveData(uart);
    }
}

void mcu_usart3_irq_route(void)
{
    serial_irq_route_byte(USART3, battery_port_rx_byte_isr);
}

void mcu_uart4_irq_route(void)
{
    serial_irq_route_byte(UART4, display_port_rx_byte_isr);
}

void mcu_uart5_irq_route(void)
{
    serial_irq_route_byte(UART5, ibus_port_rx_byte_isr);
}
