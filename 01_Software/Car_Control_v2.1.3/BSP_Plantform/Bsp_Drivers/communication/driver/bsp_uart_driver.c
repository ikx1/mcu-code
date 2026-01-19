/**
 * @file bsp_driver_uart.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bsp_uart_driver.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
/* 串口设备实例 */
uart_device_t s_uart_dev[1] = {0};

/* 串口缓存 */
static uint8_t s_uart1_tx_buf[UART1_TX_BUF_SIZE];
static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];
static uint8_t s_uart1_dmarx_buf[UART1_DMA_RX_BUF_SIZE];
static uint8_t s_uart1_dmatx_buf[UART1_DMA_TX_BUF_SIZE];

extern DMA_HandleTypeDef hdma_usart1_rx;

/********************************** Functions ********************************/
/**
 * @brief  uart1 dma发送通道配置
 * @param  
 * @retval 
 */
void uart1_dmatx_config(uint8_t *mem_addr, uint32_t mem_size)
{
	 // 关闭 DMA 流
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_7));

    // 设置通道
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);

    // 配置地址和数据长度
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)mem_addr);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)&USART1->DR);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, mem_size);

    // 配置方向：内存 -> 外设
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    // 配置优先级、中断
	LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

    // 启动 DMA 发送
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);

    // 使能 USART1 的 DMA 发送请求
    LL_USART_EnableDMAReq_TX(USART1);
}

/**
 * @brief  uart1 dma接收通道配置
 * @param  
 * @retval 
 */
void uart1_dmarx_config(uint8_t *mem_addr, uint32_t mem_size)
{
	 // 关闭 DMA 流
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
    while (LL_DMA_IsEnabledStream(DMA2, LL_DMA_STREAM_2));

    // 设置通道
    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);

    // 配置地址和数据长度
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)mem_addr);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&USART1->DR);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, mem_size);

    // 配置方向：外设 -> 内存
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    // 配置模式为循环模式
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);

    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);

    // 启动 DMA 接收
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

    // 使能 USART1 的 DMA 接收请求
    LL_USART_EnableDMAReq_RX(USART1);
}

/**
 * @brief  获取DMA接收buf剩余空间
 * @param  
 * @retval 
 */
uint16_t uart1_get_dmarx_buf_remain_size(void)
{
	 return LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);
}

/**
 * @brief 串口设备初始化 
 * @param  
 * @retval 
 */
void uart_device_init(uint8_t uart_id)
{
  	if (uart_id == 0)
	{
		/* 配置串口1收发fifo */
		fifo_register(&s_uart_dev[uart_id].tx_fifo, &s_uart1_tx_buf[0], 
                      sizeof(s_uart1_tx_buf), NULL, NULL);
		fifo_register(&s_uart_dev[uart_id].rx_fifo, &s_uart1_rx_buf[0], 
                      sizeof(s_uart1_rx_buf), NULL, NULL);
		
		/* 配置串口1 DMA收发buf */
		s_uart_dev[uart_id].dmarx_buf = &s_uart1_dmarx_buf[0];
		s_uart_dev[uart_id].dmarx_buf_size = sizeof(s_uart1_dmarx_buf);
		s_uart_dev[uart_id].dmatx_buf = &s_uart1_dmatx_buf[0];
		s_uart_dev[uart_id].dmatx_buf_size = sizeof(s_uart1_dmatx_buf);
		uart1_dmarx_config(s_uart_dev[uart_id].dmarx_buf, 
							   sizeof(s_uart1_dmarx_buf));/* 只需配置接收模式DMA，发送模式需发送数据时才配置 */
		s_uart_dev[uart_id].status  = 0;
	}
}

/**
 * @brief 串口数据写入缓冲区
 * 
 * @param uart_id 
 * @param buf 
 * @param size 
 * @return uint16_t 
 */
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size)
{
    return fifo_write(&s_uart_dev[uart_id].tx_fifo, buf, size);
}

/**
 * @brief 串口缓冲区读取数据
 * 
 * @param uart_id 
 * @param buf 
 * @param size 
 * @return uint16_t 
 */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size)
{
    return fifo_read(&s_uart_dev[uart_id].rx_fifo, buf, size);
}

/**
 * @brief 串口从缓冲区读取数据
 * 
 * @param uart_id 
 */
void uart_poll_dma_tx(uint8_t uart_id)
{
  	uint16_t size = 0;
	
	if (0x01 == s_uart_dev[uart_id].status)
    {
        return;
    }
	size = fifo_read(&s_uart_dev[uart_id].tx_fifo, s_uart_dev[uart_id].dmatx_buf,
					 s_uart_dev[uart_id].dmatx_buf_size);
	if (size != 0)
	{	
	  	if (uart_id == 0)
		{
            s_uart_dev[uart_id].status = 0x01;	/* DMA发送状态 */
		  	uart1_dmatx_config(s_uart_dev[uart_id].dmatx_buf, size);
		}
	}
}

/**
 * @brief 清楚标志位，设置为空闲
 * 
 * @param uart_id 
 */
void uart_dmatx_done_isr(uint8_t uart_id)
{
    s_uart_dev[uart_id].status = 0;
}

/**
 * @brief DMA全满中断调用
 * 
 * @param uart_id 
 */
void uart_dmarx_done_isr(uint8_t uart_id)
{
  	uint16_t recv_size;
	
	recv_size = s_uart_dev[uart_id].dmarx_buf_size - s_uart_dev[uart_id].last_dmarx_size;
	fifo_write(&s_uart_dev[uart_id].rx_fifo, 
				   (const uint8_t *)&(s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);

	s_uart_dev[uart_id].last_dmarx_size = 0;
}

/**
 * @brief DMA半满中断调用，
 * 
 * @param uart_id 
 */
void uart_dmarx_half_done_isr(uint8_t uart_id)
{
  	uint16_t recv_total_size;
  	uint16_t recv_size;
	
	if(uart_id == 0)
	{
	  	recv_total_size = s_uart_dev[uart_id].dmarx_buf_size - uart1_get_dmarx_buf_remain_size();
	}
	recv_size = recv_total_size - s_uart_dev[uart_id].last_dmarx_size;
    
	fifo_write(&s_uart_dev[uart_id].rx_fifo, 
				   (const uint8_t *)&(s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);
	s_uart_dev[uart_id].last_dmarx_size = recv_total_size;
}

/**
 * @brief 在串口空闲中断中调用，
 * 
 * @param uart_id 
 */
void uart_dmarx_idle_isr(uint8_t uart_id)
{
  	uint16_t recv_total_size;
  	uint16_t recv_size;
	
	if(uart_id == 0)
	{
	  	recv_total_size = s_uart_dev[uart_id].dmarx_buf_size - uart1_get_dmarx_buf_remain_size();
	}
	recv_size = recv_total_size - s_uart_dev[uart_id].last_dmarx_size;
	fifo_write(&s_uart_dev[uart_id].rx_fifo, 
				   (const uint8_t *)&(s_uart_dev[uart_id].dmarx_buf[s_uart_dev[uart_id].last_dmarx_size]), recv_size);
	s_uart_dev[uart_id].last_dmarx_size = recv_total_size;
}



