/**
 * @file uart_rtos.c
 * @brief Implementation of FreeRTOS adapter for DMA+FIFO UART.
 */
#include "uart_rtos.h"
#include <string.h>

/* --------- Local types --------- */
typedef struct
{
    uart_rtos_cfg_t cfg;
    TaskHandle_t tx_task;
    TaskHandle_t rx_task;
    bool inited;
} uart_port_ctx_t;

/* Supports up to 2 UARTs (F103VET6 has 5; extend if needed) */
#ifndef UART_RTOS_MAX_PORTS
#define UART_RTOS_MAX_PORTS  2
#endif

/* Shared DMA service task */
static TaskHandle_t s_service_task = NULL;
static const uint16_t s_service_stack_words_default = 256;

/* Ports context */
static uart_port_ctx_t s_ports[UART_RTOS_MAX_PORTS] = {0};

/* --------- Forward declarations --------- */
static void prvUartTxTask(void *arg);
static void prvUartRxTask(void *arg);
static void prvUartServiceTask(void *arg);

static bool uart_rtos_tx_enabled(const uart_port_ctx_t *port)
{
    return (port != NULL) && (port->cfg.build_tx != NULL);
}

static bool uart_rtos_rx_enabled(const uart_port_ctx_t *port)
{
    return (port != NULL) && (port->cfg.on_rx != NULL);
}

static uart_port_ctx_t* alloc_slot(void)
{
    for (int i = 0; i < UART_RTOS_MAX_PORTS; ++i)
    {
        if (!s_ports[i].inited)
            return &s_ports[i];
    }
    return NULL;
}

static void cleanup_port(uart_port_ctx_t *port)
{
    if (!port) return;
    if (port->tx_task) {
        vTaskDelete(port->tx_task);
        port->tx_task = NULL;
    }
    if (port->rx_task) {
        vTaskDelete(port->rx_task);
        port->rx_task = NULL;
    }
    memset(port, 0, sizeof(*port));
}

/* --------- Public API --------- */
bool uart_rtos_start(const uart_rtos_cfg_t *cfg)
{
    if (cfg == NULL || (cfg->build_tx == NULL && cfg->on_rx == NULL))
        return false;

    /* Allocate slot */
    uart_port_ctx_t *port = alloc_slot();
    if (!port) return false;

    /* Copy cfg and apply defaults */
    memset(port, 0, sizeof(*port));
    port->cfg = *cfg;
    if (uart_rtos_tx_enabled(port)) {
        if (port->cfg.tx_period_ms == 0) port->cfg.tx_period_ms = 20;
        if (port->cfg.tx_task_stack_words == 0) port->cfg.tx_task_stack_words = 256;
        if (port->cfg.tx_task_prio == 0) port->cfg.tx_task_prio = tskIDLE_PRIORITY + 2;
    }
    if (uart_rtos_rx_enabled(port)) {
        if (port->cfg.rx_period_ms == 0) port->cfg.rx_period_ms = 20;
        if (port->cfg.rx_chunk_max == 0) port->cfg.rx_chunk_max = UART_RX_BUF_SIZE;
        if (port->cfg.rx_task_stack_words == 0) port->cfg.rx_task_stack_words = 256;
        if (port->cfg.rx_task_prio == 0) port->cfg.rx_task_prio = tskIDLE_PRIORITY + 2;
    }

//    /* Bring-up BSP & device (idempotent in user's drivers) */
//    if (port->cfg.uart_id == DEV_UART1) { uart1_init(); }
    /* Add more UARTx init as needed */
    uart_device_init(port->cfg.uart_id);
    if (!uart_device_is_ready(port->cfg.uart_id))
    {
        memset(port, 0, sizeof(*port));
        return false;
    }

    /* Create per-port TX/RX tasks */
    BaseType_t ok;
    if (uart_rtos_tx_enabled(port)) {
        ok = xTaskCreate(prvUartTxTask, "UART_TX",
                         port->cfg.tx_task_stack_words,
                         port, port->cfg.tx_task_prio, &port->tx_task);
        if (ok != pdPASS) {
            cleanup_port(port);
            return false;
        }
    }

    if (uart_rtos_rx_enabled(port)) {
        ok = xTaskCreate(prvUartRxTask, "UART_RX",
                         port->cfg.rx_task_stack_words,
                         port, port->cfg.rx_task_prio, &port->rx_task);
        if (ok != pdPASS) {
            cleanup_port(port);
            return false;
        }
    }

    /* Create a single service task for all ports (once) */
    if (uart_rtos_tx_enabled(port) && s_service_task == NULL)
    {
        ok = xTaskCreate(prvUartServiceTask, "UART_DMA_SVC",
                         s_service_stack_words_default,
                         NULL, tskIDLE_PRIORITY + 1, &s_service_task);
        if (ok != pdPASS) {
            s_service_task = NULL;
            cleanup_port(port);
            return false;
        }
    }

    port->inited = true;
    return true;
}

void uart_rtos_kick_service(void)
{
    /* Wake the shared DMA pump early when TX just queued more bytes. */
    if (s_service_task)
        xTaskNotifyGive(s_service_task);
}

/* --------- Tasks --------- */
static void prvUartTxTask(void *arg)
{
    uart_port_ctx_t *port = (uart_port_ctx_t*)arg;
    const TickType_t period = pdMS_TO_TICKS(port->cfg.tx_period_ms);
    TickType_t next = xTaskGetTickCount();

    /* Local frame buffer */
    uint8_t  frame[UART_TX_BUF_SIZE];
    uint16_t pending_len = 0;
    uint16_t pending_off = 0;

    for (;;)
    {
        /* Finish the previous frame before asking build_tx() for a new one so
         * callers never overwrite an in-flight buffer with fresh telemetry. */
        if (pending_len > pending_off)
        {
            uint16_t left = (uint16_t)(pending_len - pending_off);
            uint16_t w = uart_write(port->cfg.uart_id, &frame[pending_off], left);
            pending_off = (uint16_t)(pending_off + w);

            uart_rtos_kick_service();
            vTaskDelayUntil(&next, period);
            continue;
        }

        /* Build a fresh frame only when no prior payload remains pending. */
        uint16_t len = port->cfg.build_tx(port->cfg.uart_id, frame, sizeof(frame), port->cfg.user_ctx);
        if (len > sizeof(frame))
        {
            /* Callback violated maxlen contract; drop this frame and record once. */
            uart_note_tx_build_oversize(port->cfg.uart_id);
            len = 0;
        }
        if (len > 0)
        {
            pending_len = len;
            pending_off = 0;

            /* Push once immediately, then let the shared service task continue DMA pumping. */
            uint16_t w = uart_write(port->cfg.uart_id, frame, len);
            pending_off = w;

            uart_rtos_kick_service();
        }

        vTaskDelayUntil(&next, period);
    }
}

static void prvUartRxTask(void *arg)
{
    uart_port_ctx_t *port = (uart_port_ctx_t*)arg;
    const TickType_t period = pdMS_TO_TICKS(port->cfg.rx_period_ms);
    TickType_t next = xTaskGetTickCount();

    uint8_t rx_chunk[UART_RX_BUF_SIZE];
    uint16_t chunk_max = port->cfg.rx_chunk_max;
    if (chunk_max == 0) chunk_max = UART_RX_BUF_SIZE;
    if (chunk_max > sizeof(rx_chunk)) chunk_max = sizeof(rx_chunk);

    for (;;)
    {
        /* RX task only batches raw bytes into the upper parser; protocol framing
         * and command decoding stay out of this thread. */
        uint16_t got = uart_read(port->cfg.uart_id, rx_chunk, chunk_max);
        if (got > 0)
        {
            port->cfg.on_rx(port->cfg.uart_id, rx_chunk, got, port->cfg.user_ctx);
        }
        vTaskDelayUntil(&next, period);
    }
}

static void prvUartServiceTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        /* Shared DMA service pump for every configured UART TX port. */
        for (int i = 0; i < UART_RTOS_MAX_PORTS; ++i)
        {
            if (s_ports[i].inited)
            {
                uart_poll_dma_tx(s_ports[i].cfg.uart_id);
            }
        }

        /* Wake on a kick when new TX data arrives, but keep polling periodically
         * so long DMA bursts continue even without additional notifications. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));  /* wake early on kick */
    }
}
