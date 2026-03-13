/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c
  * @brief   Main interrupt service routines.
  ******************************************************************************
  */

#include "stm32f10x_it.h"

#include "can_irq_adapter.h"
#include "serial_irq_router.h"
#include "uart_irq_adapter.h"

#include "stm32f10x_usart.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void DebugMon_Handler(void)
{
}

void USART1_IRQHandler(void)
{
    mcu_uart1_usart_irq_adapter();
}

void DMA1_Channel4_IRQHandler(void)
{
    mcu_uart1_dma_tx_irq_adapter();
}

void DMA1_Channel5_IRQHandler(void)
{
    mcu_uart1_dma_rx_irq_adapter();
}

void USART3_IRQHandler(void)
{
    mcu_usart3_irq_route();
}

void UART4_IRQHandler(void)
{
    mcu_uart4_irq_route();
}

void UART5_IRQHandler(void)
{
    mcu_uart5_irq_route();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    mcu_can1_rx0_irq_adapter();
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
    mcu_can1_tx_irq_adapter();
}

void CAN1_SCE_IRQHandler(void)
{
    mcu_can1_sce_irq_adapter();
}

void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART2);
    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        (void)USART_ReceiveData(USART2);
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_IDLE) != RESET)
    {
        (void)USART_ReceiveData(USART2);
    }
}
