
/**
 * @file uart_rtos.h
 * @brief FreeRTOS adapter for the existing DMA+FIFO UART (dev_uart/bsp_uart).
 *
 * Features:
 *  - Optional TX task at fixed period (e.g., 50 Hz)
 *  - Optional RX task at fixed period (e.g., 50 Hz)
 *  - A lightweight DMA service task that pumps uart_poll_dma_tx() when TX is enabled
 *  - No dynamic allocation inside the module (all stacks configured by caller)
 *  - No FreeRTOS API in any IRQ; ISR remains in dev_uart/bsp_uart
 *  - High cohesion: all OS-related logic is here; BSP/dev remain unchanged
 *  - Low coupling: user code communicates via two callbacks
 */
#ifndef UART_RTOS_H
#define UART_RTOS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* dev layer */
#include "bsp_uart_driver.h"

/* ---------- User callback contracts ---------- */
typedef uint16_t (*uart_user_build_tx_fn)(uint8_t uart_id,
                                          uint8_t *out, uint16_t maxlen,
                                          void *user_ctx);
/* Called from RX task (thread context) at rx_period_ms cadence */
typedef void (*uart_user_on_rx_bytes_fn)(uint8_t uart_id,
                                         const uint8_t *data, uint16_t len,
                                         void *user_ctx);

/* ---------- Configuration per UART port ---------- */
typedef struct
{
    uint8_t  uart_id;            /* DEV_UART1 / DEV_UART2 ... */
    uint32_t tx_period_ms;       /* e.g., 20 for 50 Hz */
    uint32_t rx_period_ms;       /* e.g., 20 for 50 Hz */
    uint16_t rx_chunk_max;       /* max bytes pulled from RX FIFO per period */
    /* callbacks */
    uart_user_build_tx_fn  build_tx;
    uart_user_on_rx_bytes_fn on_rx;
    void *user_ctx;
    /* RTOS resources (optional overrides) */
    uint16_t tx_task_stack_words;    /* default if 0: 256 */
    UBaseType_t tx_task_prio;        /* default if 0: tskIDLE_PRIORITY+2 */
    uint16_t rx_task_stack_words;    /* default if 0: 256 */
    UBaseType_t rx_task_prio;        /* default if 0: tskIDLE_PRIORITY+2 */
} uart_rtos_cfg_t;

/* Create enabled tasks for this UART and (when TX is enabled) a shared DMA service task */
bool uart_rtos_start(const uart_rtos_cfg_t *cfg);

/* Optional: request an immediate TX pump (e.g., after a burst write) */
void uart_rtos_kick_service(void);

#ifdef __cplusplus
}
#endif
#endif /* UART_RTOS_H */

