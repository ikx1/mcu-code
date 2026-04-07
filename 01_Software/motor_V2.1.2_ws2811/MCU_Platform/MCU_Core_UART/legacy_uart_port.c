#include "legacy_uart_port.h"

#include <stddef.h>

#include "main.h"
#include "usart.h"

#define LEGACY_UART_PORT_TX_TIMEOUT_MS 100u
#define LEGACY_UART_RX_FIFO_SIZE       256u

typedef struct
{
    uint8_t buf[LEGACY_UART_RX_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} legacy_uart_rx_fifo_t;

static legacy_uart_rx_byte_cb_t s_battery_rx_cb = NULL;
static legacy_uart_rx_byte_cb_t s_display_rx_cb = NULL;
static legacy_uart_rx_byte_cb_t s_ibus_rx_cb = NULL;
static legacy_uart_rx_fifo_t s_battery_rx_fifo = {0};
static legacy_uart_rx_fifo_t s_display_rx_fifo = {0};
static legacy_uart_rx_fifo_t s_ibus_rx_fifo = {0};

static void legacy_uart_rx_fifo_push_isr(legacy_uart_rx_fifo_t *fifo, uint8_t byte)
{
    uint16_t head;
    uint16_t next;

    if (fifo == NULL)
    {
        return;
    }

    head = fifo->head;
    next = (uint16_t)((head + 1u) % LEGACY_UART_RX_FIFO_SIZE);
    if (next == fifo->tail)
    {
        return;
    }

    fifo->buf[head] = byte;
    fifo->head = next;
}

static uint8_t legacy_uart_rx_fifo_pop(legacy_uart_rx_fifo_t *fifo, uint8_t *out_byte)
{
    uint16_t tail;

    if ((fifo == NULL) || (out_byte == NULL))
    {
        return 0u;
    }

    if (fifo->tail == fifo->head)
    {
        return 0u;
    }

    tail = fifo->tail;
    *out_byte = fifo->buf[tail];
    fifo->tail = (uint16_t)((tail + 1u) % LEGACY_UART_RX_FIFO_SIZE);

    return 1u;
}

static void ultrasonic_port_flush_rx(void)
{
    while (LL_USART_IsActiveFlag_RXNE(UART8))
    {
        (void)UART8->DR;
    }
}

void battery_port_init(void)
{
    s_battery_rx_fifo.head = 0u;
    s_battery_rx_fifo.tail = 0u;
    HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, GPIO_PIN_RESET);
}

void battery_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    s_battery_rx_cb = cb;
}

uint16_t battery_port_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == NULL || len == 0u) {
        return 0u;
    }

    HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, GPIO_PIN_SET);

    for (i = 0u; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART3)) {
        }
        LL_USART_TransmitData8(USART3, buf[i]);
    }

    while (!LL_USART_IsActiveFlag_TC(USART3)) {
    }

    HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, GPIO_PIN_RESET);
    return len;
}

void battery_port_rx_byte_isr(uint8_t byte)
{
    legacy_uart_rx_fifo_push_isr(&s_battery_rx_fifo, byte);
}

void battery_port_poll_rx(void)
{
    uint8_t byte = 0u;
    legacy_uart_rx_byte_cb_t cb = s_battery_rx_cb;

    while (legacy_uart_rx_fifo_pop(&s_battery_rx_fifo, &byte) != 0u)
    {
        if (cb != NULL)
        {
            cb(byte);
        }
    }
}

void display_port_init(void)
{
    s_display_rx_fifo.head = 0u;
    s_display_rx_fifo.tail = 0u;
}

void display_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    s_display_rx_cb = cb;
}

uint16_t display_port_write(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == NULL || len == 0u) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART2)) {
        }
        LL_USART_TransmitData8(USART2, buf[i]);
    }

    while (!LL_USART_IsActiveFlag_TC(USART2)) {
    }

    return len;
}

void display_port_rx_byte_isr(uint8_t byte)
{
    legacy_uart_rx_fifo_push_isr(&s_display_rx_fifo, byte);
}

void display_port_poll_rx(void)
{
    uint8_t byte = 0u;
    legacy_uart_rx_byte_cb_t cb = s_display_rx_cb;

    while (legacy_uart_rx_fifo_pop(&s_display_rx_fifo, &byte) != 0u)
    {
        if (cb != NULL)
        {
            cb(byte);
        }
    }
}

void ibus_port_init(void)
{
    s_ibus_rx_fifo.head = 0u;
    s_ibus_rx_fifo.tail = 0u;
}

void ibus_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    s_ibus_rx_cb = cb;
}

void ibus_port_rx_byte_isr(uint8_t byte)
{
    legacy_uart_rx_fifo_push_isr(&s_ibus_rx_fifo, byte);
}

void ibus_port_poll_rx(void)
{
    uint8_t byte = 0u;
    legacy_uart_rx_byte_cb_t cb = s_ibus_rx_cb;

    while (legacy_uart_rx_fifo_pop(&s_ibus_rx_fifo, &byte) != 0u)
    {
        if (cb != NULL)
        {
            cb(byte);
        }
    }
}

void ultrasonic_port_init(void)
{
    HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, GPIO_PIN_RESET);
}

uint16_t ultrasonic_port_write(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) {
        return 0u;
    }

    ultrasonic_port_flush_rx();
    HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, GPIO_PIN_SET);

    if (HAL_UART_Transmit(&huart8, (uint8_t *)buf, len, LEGACY_UART_PORT_TX_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, GPIO_PIN_RESET);
        return 0u;
    }

    HAL_GPIO_WritePin(RS485_2_RW_GPIO_Port, RS485_2_RW_Pin, GPIO_PIN_RESET);
    return len;
}

uint16_t ultrasonic_port_read(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0u) {
        return 0u;
    }

    if (HAL_UART_Receive(&huart8, buf, len, timeout_ms) != HAL_OK) {
        return 0u;
    }

    return len;
}
