#include "uart_port.h"

#include <stddef.h>

#include "bsp_uart_driver.h"
#include "uart.h"

#include "stm32f10x_dma.h"

bool uart_port_dma_tx_start(uint8_t uart_id, const uint8_t *mem_addr, uint16_t mem_size)
{
    if ((uart_id != DEV_UART1) || (mem_addr == NULL) || (mem_size == 0u))
    {
        return false;
    }

    uart1_dmatx_config((uint8_t *)mem_addr, mem_size);
    return true;
}

bool uart_port_dma_rx_start(uint8_t uart_id, uint8_t *mem_addr, uint16_t mem_size)
{
    if ((uart_id != DEV_UART1) || (mem_addr == NULL) || (mem_size == 0u))
    {
        return false;
    }

    uart1_dmarx_config(mem_addr, mem_size);
    return true;
}

uint16_t uart_port_dma_rx_get_remain(uint8_t uart_id)
{
    if (uart_id != DEV_UART1)
    {
        return 0u;
    }

    return uart1_get_dmarx_buf_remain_size();
}

void uart_port_dma_tx_stop(uint8_t uart_id)
{
    if (uart_id != DEV_UART1)
    {
        return;
    }

    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL4 | DMA1_FLAG_TC4 | DMA1_FLAG_HT4 | DMA1_FLAG_TE4);
}
