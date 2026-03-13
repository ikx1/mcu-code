#include "uart_irq_adapter.h"

#include "bsp_uart_driver.h"

#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"

void mcu_uart1_usart_irq_adapter(void)
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        uart_dmarx_idle_isr(DEV_UART1);
        (void)USART_ReceiveData(USART1);
    }
}

void mcu_uart1_dma_rx_irq_adapter(void)
{
    if (DMA_GetITStatus(DMA1_IT_HT5) != RESET)
    {
        uart_dmarx_half_done_isr(DEV_UART1);
        DMA_ClearFlag(DMA1_FLAG_HT5);
    }

    if (DMA_GetITStatus(DMA1_IT_TC5) != RESET)
    {
        uart_dmarx_done_isr(DEV_UART1);
        DMA_ClearFlag(DMA1_FLAG_TC5);
    }
}

void mcu_uart1_dma_tx_irq_adapter(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        uart_dmatx_done_isr(DEV_UART1);
        DMA_ClearFlag(DMA1_FLAG_TC4);
        DMA_Cmd(DMA1_Channel4, DISABLE);
    }
}
