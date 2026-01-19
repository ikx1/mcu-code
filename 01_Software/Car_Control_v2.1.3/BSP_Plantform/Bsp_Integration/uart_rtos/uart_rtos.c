
/**
 * @file uart_rtos.c
 * @brief Implementation of FreeRTOS adapter for DMA+FIFO UART.
 */
#include "uart_rtos.h"
#include <string.h>
#include "usart.h"

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
    if (!cfg || !cfg->build_tx || !cfg->on_rx)
        return false;

    /* Allocate slot */
    uart_port_ctx_t *port = alloc_slot();
    if (!port) return false;

    /* Copy cfg and apply defaults */
    memset(port, 0, sizeof(*port));
    port->cfg = *cfg;
    if (port->cfg.tx_period_ms == 0) port->cfg.tx_period_ms = 20;
    if (port->cfg.rx_period_ms == 0) port->cfg.rx_period_ms = 20;
    if (port->cfg.rx_chunk_max == 0) port->cfg.rx_chunk_max = 256;
    if (port->cfg.tx_task_stack_words == 0) port->cfg.tx_task_stack_words = 256;
    if (port->cfg.rx_task_stack_words == 0) port->cfg.rx_task_stack_words = 256;
    if (port->cfg.tx_task_prio == 0) port->cfg.tx_task_prio = tskIDLE_PRIORITY + 2;
    if (port->cfg.rx_task_prio == 0) port->cfg.rx_task_prio = tskIDLE_PRIORITY + 2;

//    /* Bring-up BSP & device (idempotent in user's drivers) */
//    if (port->cfg.uart_id == DEV_UART1) { uart1_init(); }
    /* Add more UARTx init as needed */
    uart_device_init(port->cfg.uart_id);

    /* Create per-port TX/RX tasks */
    BaseType_t ok;
    ok = xTaskCreate(prvUartTxTask, "UART_TX",
                     port->cfg.tx_task_stack_words,
                     port, port->cfg.tx_task_prio, &port->tx_task);
    if (ok != pdPASS) {
        cleanup_port(port);
        return false;
    }

    ok = xTaskCreate(prvUartRxTask, "UART_RX",
                     port->cfg.rx_task_stack_words,
                     port, port->cfg.rx_task_prio, &port->rx_task);
    if (ok != pdPASS) {
        cleanup_port(port);
        return false;
    }

    /* Create a single service task for all ports (once) */
    if (s_service_task == NULL)
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
    uint8_t  frame[256];
    uint16_t pending_len = 0;
    uint16_t pending_off = 0;

    for (;;)
    {
        /* 1) 如果上一帧没写完，先续写，禁止构建新帧 */
        if (pending_len > pending_off)
        {
            uint16_t left = (uint16_t)(pending_len - pending_off);
            uint16_t w = uart_write(port->cfg.uart_id, &frame[pending_off], left);
            pending_off = (uint16_t)(pending_off + w);

            uart_rtos_kick_service();

            /* 没写完就下个周期继续 */
            vTaskDelayUntil(&next, period);
            continue;
        }

        /* 2) 构建新帧 */
        uint16_t len = port->cfg.build_tx(port->cfg.uart_id, frame, sizeof(frame), port->cfg.user_ctx);
        if (len > 0)
        {
            pending_len = len;
            pending_off = 0;

            /* 先尝试写一次 */
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

    uint8_t rx_chunk[512];
    uint16_t chunk_max = port->cfg.rx_chunk_max;
    if (chunk_max == 0) chunk_max = 256;
    if (chunk_max > sizeof(rx_chunk)) chunk_max = sizeof(rx_chunk);

    for (;;)
    {
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
        /* Pump all configured ports */
        for (int i = 0; i < UART_RTOS_MAX_PORTS; ++i)
        {
            if (s_ports[i].inited)
            {
                uart_poll_dma_tx(s_ports[i].cfg.uart_id);
            }
        }

        /* Wait briefly or until kicked */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));  /* wake early on kick */
        /* Even if timeout, loop again to pump regularly */
    }
}
