#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifndef UART_DEV_MAX
#define UART_DEV_MAX                 1u
#endif

#ifndef UART_TX_BUF_SIZE
#define UART_TX_BUF_SIZE             256u
#endif
#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE             256u
#endif
#ifndef UART_DMA_RX_BUF_SIZE
#define UART_DMA_RX_BUF_SIZE         256u
#endif
#ifndef UART_DMA_TX_BUF_SIZE
#define UART_DMA_TX_BUF_SIZE         256u
#endif

/* Keep compatibility with old macro names. */
#ifndef UART1_TX_BUF_SIZE
#define UART1_TX_BUF_SIZE            UART_TX_BUF_SIZE
#endif
#ifndef UART1_RX_BUF_SIZE
#define UART1_RX_BUF_SIZE            UART_RX_BUF_SIZE
#endif
#ifndef UART1_DMA_RX_BUF_SIZE
#define UART1_DMA_RX_BUF_SIZE        UART_DMA_RX_BUF_SIZE
#endif
#ifndef UART1_DMA_TX_BUF_SIZE
#define UART1_DMA_TX_BUF_SIZE        UART_DMA_TX_BUF_SIZE
#endif

#define DEV_UART1                    0u
#define DEV_UART2                    1u

typedef enum
{
    UART_OK = 0,
    UART_ERROR_TIMEOUT,
    UART_ERROR_INVALID_PARAM,
    UART_ERROR_BUFFER_FULL,
    UART_ERROR_BUFFER_EMPTY,
    UART_ERROR_DMA_ERROR,
} bsp_uart_status_t;

typedef struct
{
    uint32_t tx_req_bytes;
    uint32_t tx_queued_bytes;
    uint32_t tx_drop_bytes;
    uint32_t tx_dma_start_fail_cnt;
    uint32_t tx_busy_skip_cnt;
    uint32_t tx_build_oversize_cnt;

    uint32_t rx_isr_bytes;
    uint32_t rx_queued_bytes;
    uint32_t rx_drop_bytes;
    uint32_t rx_overflow_cnt;
    uint32_t rx_bad_isr_state_cnt;
} uart_diag_t;

void uart_device_init(uint8_t uart_id);
bool uart_device_is_ready(uint8_t uart_id);

uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size);
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size);

void uart_dmarx_done_isr(uint8_t uart_id);
void uart_dmarx_half_done_isr(uint8_t uart_id);
void uart_dmarx_idle_isr(uint8_t uart_id);
void uart_dmatx_done_isr(uint8_t uart_id);

void uart_poll_dma_tx(uint8_t uart_id);

void uart_get_diag(uint8_t uart_id, uart_diag_t *out_diag);
void uart_clear_diag(uint8_t uart_id);
void uart_note_tx_build_oversize(uint8_t uart_id);

#endif /* UART_DRIVER_H */
