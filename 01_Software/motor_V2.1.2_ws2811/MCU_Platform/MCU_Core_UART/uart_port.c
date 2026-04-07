#include "uart_port.h"

#include <stdint.h>

#include "bsp_uart_driver.h"
#include "usart.h"

/**
 * @brief 启动UART1 DMA发送
 * 
 * @param uart_id UART1
 * @param mem_addr 数据内存地址
 * @param mem_size 数据大小
 * @return true 
 * @return false 
 */
bool uart_port_dma_tx_start(uint8_t uart_id, const uint8_t *mem_addr, uint16_t mem_size)
{
    if (uart_id != DEV_UART1 || mem_addr == NULL || mem_size == 0u) {
        return false;
    }

    /* 禁用DMA 2 流 7 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7)) {
    }

    /* 配置DMA 2 流 7 为通道 4 */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
    /* 配置DMA 2 流 7 内存地址 */
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)(uintptr_t)mem_addr);
    /* 配置DMA 2 流 7 外设地址 */
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)(uintptr_t)&USART1->DR);
    /* 配置DMA 2 流 7 数据长度 */
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, mem_size);

    /* 配置DMA 2 流 7 数据传输方向为内存到外设 */
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    /* 配置DMA 2 流 7 优先级为中 */
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);
    /* 配置DMA 2 流 7 模式为普通 */
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
    /* 配置DMA 2 流 7 外设增量模式为不增量 */
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    /* 配置DMA 2 流 7 内存增量模式为增量 */
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    /* 配置DMA 2 流 7 外设数据大小为字节 */
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    /* 配置DMA 2 流 7 内存数据大小为字节 */
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

    /* 启用DMA 2 流 7 */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
    /* 启用UART1 DMA发送请求 */
    LL_USART_EnableDMAReq_TX(USART1);
    return true;
}

/**
 * @brief 启动UART1 DMA接收
 * 
 * @param uart_id UART1
 * @param mem_addr 数据内存地址
 * @param mem_size 数据大小
 * @return true 
 * @return false 
 */
bool uart_port_dma_rx_start(uint8_t uart_id, uint8_t *mem_addr, uint16_t mem_size)
{
    if (uart_id != DEV_UART1 || mem_addr == NULL || mem_size == 0u) {
        return false;
    }

    /* 禁用DMA 2 流 2 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
    /* 等待DMA2流2禁用 */
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2)) {
    }

    /* 配置DMA 2 流通道 4 */
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
    /* 配置DMA 2 流 2 内存地址 */
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)(uintptr_t)mem_addr);
    /* 配置DMA 2 流 2 外设地址 */
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)(uintptr_t)&USART1->DR);
    /* 配置DMA 2 流 2 数据长度 */
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, mem_size);

    /* 配置DMA 2 流 2 数据传输方向为外设到内存 */
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    /* 配置DMA 2 流 2 模式为循环 */
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
    /* 配置DMA 2 流 2 外设增量模式为不增量 */
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
    /* 配置DMA 2 流 2 内存增量模式为增量 */
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
    /* 配置DMA 2 流 2 外设数据大小为字节 */
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
    /* 配置DMA 2 流 2 内存数据大小为字节 */
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
    /* 配置DMA 2 流 2 优先级为高 */
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);

    /* 启用DMA 2 流 2 */
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
    /* 启用UART1 DMA接收请求 */
    LL_USART_EnableDMAReq_RX(USART1);
    return true;
}

/**
 * @brief 获取UART1 DMA接收剩余数据大小
 * 
 * @param uart_id UART1
 * @return uint16_t 剩余数据大小
 */
uint16_t uart_port_dma_rx_get_remain(uint8_t uart_id)
{
    if (uart_id != DEV_UART1) {
        return 0u;
    }

    /* 获取DMA 2 流 2 剩余数据大小 */
    return (uint16_t)LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
}

/**
 * @brief 停止UART1 DMA发送
 * 
 * @param uart_id UART1
 */
void uart_port_dma_tx_stop(uint8_t uart_id)
{
    if (uart_id != DEV_UART1) {
        return;
    }

    /* 禁用DMA 2 流 7 */
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
}

