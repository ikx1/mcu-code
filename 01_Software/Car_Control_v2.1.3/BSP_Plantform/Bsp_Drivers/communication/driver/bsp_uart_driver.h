/**
 * @file bsp_driver_uart.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_UART_DRIVER_H__
#define __BSP_UART_DRIVER_H__



/********************************** Includes *********************************/
#include <stdbool.h>
#include <stdint.h>
#include "fifo.h"

/********************************** Defines **********************************/
#define UART1_TX_BUF_SIZE           256
#define UART1_RX_BUF_SIZE           256
#define	UART1_DMA_RX_BUF_SIZE		256
#define	UART1_DMA_TX_BUF_SIZE		256

#define DEV_UART1	0
#define DEV_UART2	1

/********************************** Variables ********************************/
typedef enum
{
    UART_OK = 0,
    UART_ERROR_TIMEOUT,
    UART_ERROR_INVALID_PARAM,
    UART_ERROR_BUFFER_FULL,
    UART_ERROR_BUFFER_EMPTY,
    UART_ERROR_DMA_ERROR,
}bsp_uart_status_t;

typedef struct 
{
	uint8_t status;		/* 发送状态 */
	_fifo_t tx_fifo;	/* 发送fifo */
	_fifo_t rx_fifo;	/* 接收fifo */
	uint8_t *dmarx_buf;	/* dma接收缓存 */
	uint16_t dmarx_buf_size;/* dma接收缓存大小*/
	uint8_t *dmatx_buf;	/* dma发送缓存 */
	uint16_t dmatx_buf_size;/* dma发送缓存大小 */
	uint16_t last_dmarx_size;/* dma上一次接收数据大小 */
	volatile uint8_t is_dma_tx_busy; /* DMA 发送忙标志 */
} uart_device_t;


/********************************** Functions ********************************/
extern uart_device_t s_uart_dev[1];

extern void uart_device_init(uint8_t uart_id);
extern uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size);
extern uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size);
extern void uart_dmarx_done_isr(uint8_t uart_id);
extern void uart_dmarx_half_done_isr(uint8_t uart_id);
extern void uart_dmarx_idle_isr(uint8_t uart_id);
extern void uart_dmatx_done_isr(uint8_t uart_id);
extern void uart_poll_dma_tx(uint8_t uart_id);

#endif /* __BSP_UART_DRIVER_H__ */
