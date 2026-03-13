#include "legacy_uart_port.h"

#include <stddef.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include "FreeRTOS.h"
#include "task.h"

#define LEGACY_UART_RX_FIFO_SIZE 256u

#ifndef LEGACY_UART_BATTERY_RS485_GPIO_PORT
#define LEGACY_UART_BATTERY_RS485_GPIO_PORT GPIOE
#endif

#ifndef LEGACY_UART_BATTERY_RS485_GPIO_PIN
#define LEGACY_UART_BATTERY_RS485_GPIO_PIN  GPIO_Pin_15
#endif

#ifndef LEGACY_UART_ULTRASONIC_RS485_GPIO_PORT
#define LEGACY_UART_ULTRASONIC_RS485_GPIO_PORT GPIOE
#endif

#ifndef LEGACY_UART_ULTRASONIC_RS485_GPIO_PIN
#define LEGACY_UART_ULTRASONIC_RS485_GPIO_PIN  GPIO_Pin_14
#endif

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

static void legacy_uart_rx_fifo_reset(legacy_uart_rx_fifo_t *fifo)
{
    if (fifo == NULL)
    {
        return;
    }

    fifo->head = 0u;
    fifo->tail = 0u;
}

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

static void legacy_uart_rs485_set_rx(GPIO_TypeDef *gpio_port, uint16_t gpio_pin)
{
    GPIO_ResetBits(gpio_port, gpio_pin);
}

static void legacy_uart_rs485_set_tx(GPIO_TypeDef *gpio_port, uint16_t gpio_pin)
{
    GPIO_SetBits(gpio_port, gpio_pin);
}

static void legacy_uart_send_blocking(USART_TypeDef *uart, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((uart == NULL) || (buf == NULL) || (len == 0u))
    {
        return;
    }

    for (i = 0u; i < len; i++)
    {
        while (USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(uart, buf[i]);
    }

    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
    {
    }
}

static uint16_t legacy_uart_receive_blocking(USART_TypeDef *uart,
                                             uint8_t *buf,
                                             uint16_t len,
                                             uint32_t timeout_ms)
{
    uint16_t i;
    TickType_t start_tick;
    TickType_t timeout_ticks;

    if ((uart == NULL) || (buf == NULL) || (len == 0u))
    {
        return 0u;
    }

    start_tick = xTaskGetTickCount();
    timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if ((timeout_ms != 0u) && (timeout_ticks == 0u))
    {
        timeout_ticks = 1u;
    }

    for (i = 0u; i < len; i++)
    {
        while (USART_GetFlagStatus(uart, USART_FLAG_RXNE) == RESET)
        {
            if ((xTaskGetTickCount() - start_tick) >= timeout_ticks)
            {
                return i;
            }
        }

        buf[i] = (uint8_t)USART_ReceiveData(uart);
    }

    return len;
}

static void ultrasonic_port_flush_rx(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        (void)USART_ReceiveData(USART2);
    }
}

void battery_port_init(void)
{
    legacy_uart_rx_fifo_reset(&s_battery_rx_fifo);
    legacy_uart_rs485_set_rx(LEGACY_UART_BATTERY_RS485_GPIO_PORT,
                             LEGACY_UART_BATTERY_RS485_GPIO_PIN);
}

void battery_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    s_battery_rx_cb = cb;
}

uint16_t battery_port_write(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return 0u;
    }

    legacy_uart_rs485_set_tx(LEGACY_UART_BATTERY_RS485_GPIO_PORT,
                             LEGACY_UART_BATTERY_RS485_GPIO_PIN);
    legacy_uart_send_blocking(USART3, buf, len);
    legacy_uart_rs485_set_rx(LEGACY_UART_BATTERY_RS485_GPIO_PORT,
                             LEGACY_UART_BATTERY_RS485_GPIO_PIN);

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
    legacy_uart_rx_fifo_reset(&s_display_rx_fifo);
}

void display_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    s_display_rx_cb = cb;
}

uint16_t display_port_write(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return 0u;
    }

    legacy_uart_send_blocking(UART4, buf, len);
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
    legacy_uart_rx_fifo_reset(&s_ibus_rx_fifo);
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
    legacy_uart_rs485_set_rx(LEGACY_UART_ULTRASONIC_RS485_GPIO_PORT,
                             LEGACY_UART_ULTRASONIC_RS485_GPIO_PIN);
}

uint16_t ultrasonic_port_write(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return 0u;
    }

    ultrasonic_port_flush_rx();
    legacy_uart_rs485_set_tx(LEGACY_UART_ULTRASONIC_RS485_GPIO_PORT,
                             LEGACY_UART_ULTRASONIC_RS485_GPIO_PIN);
    legacy_uart_send_blocking(USART2, buf, len);
    legacy_uart_rs485_set_rx(LEGACY_UART_ULTRASONIC_RS485_GPIO_PORT,
                             LEGACY_UART_ULTRASONIC_RS485_GPIO_PIN);

    return len;
}

uint16_t ultrasonic_port_read(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return legacy_uart_receive_blocking(USART2, buf, len, timeout_ms);
}
