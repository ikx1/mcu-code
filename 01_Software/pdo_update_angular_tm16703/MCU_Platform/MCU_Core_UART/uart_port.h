#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Platform UART DMA porting hooks.
 * Keep this interface stable; only this file + implementation should change
 * when migrating to another MCU/SDK.
 */
bool uart_port_dma_tx_start(uint8_t uart_id, const uint8_t *mem_addr, uint16_t mem_size);
bool uart_port_dma_rx_start(uint8_t uart_id, uint8_t *mem_addr, uint16_t mem_size);
uint16_t uart_port_dma_rx_get_remain(uint8_t uart_id);
void uart_port_dma_tx_stop(uint8_t uart_id);

#ifdef __cplusplus
}
#endif

#endif /* UART_PORT_H */
