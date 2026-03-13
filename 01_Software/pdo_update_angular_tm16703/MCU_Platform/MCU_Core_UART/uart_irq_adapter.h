#ifndef UART_IRQ_ADAPTER_H
#define UART_IRQ_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IRQ adapter entry points for UART1 on STM32F103.
 * Keep BSP ISR APIs behind this adapter so interrupt files stay decoupled.
 */
void mcu_uart1_usart_irq_adapter(void);
void mcu_uart1_dma_rx_irq_adapter(void);
void mcu_uart1_dma_tx_irq_adapter(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_IRQ_ADAPTER_H */
