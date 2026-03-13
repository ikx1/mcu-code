#ifndef SERIAL_IRQ_ROUTER_H
#define SERIAL_IRQ_ROUTER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Route non-stack serial IRQ bytes to legacy BSP modules.
 * Keep this separate from the UART1 communication stack adapter.
 */
void mcu_usart3_irq_route(void);
void mcu_uart4_irq_route(void);
void mcu_uart5_irq_route(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_IRQ_ROUTER_H */
