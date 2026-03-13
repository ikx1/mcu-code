#ifndef UART_LEGACY_BRIDGE_H
#define UART_LEGACY_BRIDGE_H

#include <stdint.h>

#include "legacy_uart_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Transitional facade to isolate legacy UART port API from upper modules. */
static inline void uart_legacy_battery_init(void)
{
    battery_port_init();
}

static inline uint16_t uart_legacy_battery_write(const uint8_t *buf, uint16_t len)
{
    return battery_port_write(buf, len);
}

static inline void uart_legacy_battery_rx_byte_isr(uint8_t byte)
{
    battery_port_rx_byte_isr(byte);
}

static inline void uart_legacy_battery_poll_rx(void)
{
    battery_port_poll_rx();
}

static inline void uart_legacy_battery_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    battery_port_set_rx_handler(cb);
}

static inline void uart_legacy_display_init(void)
{
    display_port_init();
}

static inline uint16_t uart_legacy_display_write(const uint8_t *buf, uint16_t len)
{
    return display_port_write(buf, len);
}

static inline void uart_legacy_display_rx_byte_isr(uint8_t byte)
{
    display_port_rx_byte_isr(byte);
}

static inline void uart_legacy_display_poll_rx(void)
{
    display_port_poll_rx();
}

static inline void uart_legacy_display_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    display_port_set_rx_handler(cb);
}

static inline void uart_legacy_ibus_init(void)
{
    ibus_port_init();
}

static inline void uart_legacy_ibus_rx_byte_isr(uint8_t byte)
{
    ibus_port_rx_byte_isr(byte);
}

static inline void uart_legacy_ibus_poll_rx(void)
{
    ibus_port_poll_rx();
}

static inline void uart_legacy_ibus_set_rx_handler(legacy_uart_rx_byte_cb_t cb)
{
    ibus_port_set_rx_handler(cb);
}

static inline void uart_legacy_ultrasonic_init(void)
{
    ultrasonic_port_init();
}

static inline uint16_t uart_legacy_ultrasonic_write(const uint8_t *buf, uint16_t len)
{
    return ultrasonic_port_write(buf, len);
}

static inline uint16_t uart_legacy_ultrasonic_read(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    return ultrasonic_port_read(buf, len, timeout_ms);
}

#ifdef __cplusplus
}
#endif

#endif /* UART_LEGACY_BRIDGE_H */
