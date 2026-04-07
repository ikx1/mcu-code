#ifndef UART_STACK_H
#define UART_STACK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t ver;
    uint8_t seq;
    uint8_t type;
    uint16_t len;
    /* Points to internal RX buffer; copy it if you need to keep it after callback returns. */
    const uint8_t *payload;
} uart_stack_frame_t;

typedef uint16_t (*uart_stack_build_tx_fn)(uint8_t uart_id,
                                           uint8_t *out,
                                           uint16_t maxlen,
                                           void *user_ctx);

typedef void (*uart_stack_on_frame_fn)(uint8_t uart_id,
                                       const uart_stack_frame_t *frame,
                                       void *user_ctx);

typedef struct
{
    uint8_t uart_id;

    uint32_t tx_period_ms;
    uint32_t rx_period_ms;
    uint16_t rx_chunk_max;

    uart_stack_build_tx_fn build_tx;
    uart_stack_on_frame_fn on_frame;
    void *user_ctx;

    uint16_t tx_task_stack_words;
    uint32_t tx_task_prio;
    uint16_t rx_task_stack_words;
    uint32_t rx_task_prio;
} uart_stack_cfg_t;

bool uart_stack_start(const uart_stack_cfg_t *cfg);
void uart_stack_kick_tx(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_STACK_H */

