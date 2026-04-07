#include "uart_stack.h"

#include <stddef.h>
#include <string.h>

#include "uart_rtos.h"
#include "host_protocol.h"

typedef struct
{
    bool used;

    uint8_t uart_id;
    uart_parser_t parser;

    uart_stack_build_tx_fn build_tx;
    uart_stack_on_frame_fn on_frame;
    void *user_ctx;
} uart_stack_port_ctx_t;

#ifndef UART_STACK_MAX_PORTS
#define UART_STACK_MAX_PORTS 2
#endif

static uart_stack_port_ctx_t s_stack_ports[UART_STACK_MAX_PORTS];

static uart_stack_port_ctx_t *uart_stack_alloc_port(void)
{
    uint32_t i;
    for (i = 0u; i < UART_STACK_MAX_PORTS; ++i)
    {
        if (!s_stack_ports[i].used) {
            return &s_stack_ports[i];
        }
    }
    return NULL;
}

static void uart_stack_dispatch_frame(uart_stack_port_ctx_t *port, const uart_frame_t *frame)
{
    uart_stack_frame_t api_frame;

    if (port == NULL || frame == NULL || port->on_frame == NULL) {
        return;
    }

    api_frame.ver = frame->ver;
    api_frame.seq = frame->seq;
    api_frame.type = frame->type;
    api_frame.len = frame->len;
    api_frame.payload = frame->payload;

    port->on_frame(port->uart_id, &api_frame, port->user_ctx);
}

static void uart_stack_feed_bytes(uart_stack_port_ctx_t *port, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (port == NULL || data == NULL || len == 0u) {
        return;
    }

    for (i = 0u; i < len; ++i)
    {
        uart_frame_t frame;
        if (uart_parser_feed(&port->parser, data[i], &frame)) {
            uart_stack_dispatch_frame(port, &frame);
        }
    }
}

static void uart_stack_on_rx_bytes(uint8_t uart_id,
                                   const uint8_t *data,
                                   uint16_t len,
                                   void *user_ctx)
{
    uart_stack_port_ctx_t *port = (uart_stack_port_ctx_t *)user_ctx;

    if (port == NULL || !port->used) {
        return;
    }
    if (uart_id != port->uart_id) {
        return;
    }

    uart_stack_feed_bytes(port, data, len);
}

static uint16_t uart_stack_build_tx_wrap(uint8_t uart_id,
                                         uint8_t *out,
                                         uint16_t maxlen,
                                         void *user_ctx)
{
    uart_stack_port_ctx_t *port = (uart_stack_port_ctx_t *)user_ctx;

    if (port == NULL || !port->used || port->build_tx == NULL) {
        return 0u;
    }

    return port->build_tx(uart_id, out, maxlen, port->user_ctx);
}

bool uart_stack_start(const uart_stack_cfg_t *cfg)
{
    uart_stack_port_ctx_t *port;
    uart_rtos_cfg_t rtos_cfg;

    if (cfg == NULL || cfg->on_frame == NULL) {
        return false;
    }

    port = uart_stack_alloc_port();
    if (port == NULL) {
        return false;
    }

    memset(port, 0, sizeof(*port));
    port->uart_id = cfg->uart_id;
    port->build_tx = cfg->build_tx;
    port->on_frame = cfg->on_frame;
    port->user_ctx = cfg->user_ctx;

    uart_parser_init(&port->parser);

    memset(&rtos_cfg, 0, sizeof(rtos_cfg));
    rtos_cfg.uart_id = cfg->uart_id;
    rtos_cfg.tx_period_ms = cfg->tx_period_ms;
    rtos_cfg.rx_period_ms = cfg->rx_period_ms;
    rtos_cfg.rx_chunk_max = cfg->rx_chunk_max;
    rtos_cfg.build_tx = (cfg->build_tx != NULL) ? uart_stack_build_tx_wrap : NULL;
    rtos_cfg.on_rx = uart_stack_on_rx_bytes;
    rtos_cfg.user_ctx = port;
    rtos_cfg.tx_task_stack_words = cfg->tx_task_stack_words;
    rtos_cfg.tx_task_prio = (UBaseType_t)cfg->tx_task_prio;
    rtos_cfg.rx_task_stack_words = cfg->rx_task_stack_words;
    rtos_cfg.rx_task_prio = (UBaseType_t)cfg->rx_task_prio;

    port->used = true;
    if (!uart_rtos_start(&rtos_cfg)) {
        memset(port, 0, sizeof(*port));
        return false;
    }

    return true;
}

void uart_stack_kick_tx(void)
{
    uart_rtos_kick_service();
}





