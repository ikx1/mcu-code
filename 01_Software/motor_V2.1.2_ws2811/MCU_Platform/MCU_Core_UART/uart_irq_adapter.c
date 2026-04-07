#include "uart_irq_adapter.h"

#include "bsp_uart_driver.h"
#include "usart.h"

void mcu_uart1_usart_irq_adapter(void)
{
    //空闲中断
    if (LL_USART_IsActiveFlag_IDLE(USART1))
    {
        uart_dmarx_idle_isr(DEV_UART1);
        LL_USART_ReceiveData8(USART1);
    }
}

void mcu_uart1_dma_rx_irq_adapter(void)
{
    //半满中断
    if (LL_DMA_IsActiveFlag_HT2(DMA2))
    {
        uart_dmarx_half_done_isr(DEV_UART1);
        LL_DMA_ClearFlag_HT2(DMA2);
    }

    //传输完成中断
    if (LL_DMA_IsActiveFlag_TC2(DMA2))
    {
        uart_dmarx_done_isr(DEV_UART1);
        LL_DMA_ClearFlag_TC2(DMA2);
    }
}

void mcu_uart1_dma_tx_irq_adapter(void)
{
    //传输完成中断
    if (LL_DMA_IsActiveFlag_TC7(DMA2))
    {
        uart_dmatx_done_isr(DEV_UART1);
        LL_DMA_ClearFlag_TC7(DMA2);
        LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
    }
}
