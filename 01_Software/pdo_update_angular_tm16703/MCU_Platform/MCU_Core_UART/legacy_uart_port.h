#ifndef LEGACY_UART_PORT_H
#define LEGACY_UART_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*legacy_uart_rx_byte_cb_t)(uint8_t byte);

void battery_port_init(void);
uint16_t battery_port_write(const uint8_t *buf, uint16_t len);
void battery_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb);
void battery_port_rx_byte_isr(uint8_t byte);
void battery_port_poll_rx(void);

void display_port_init(void);
uint16_t display_port_write(const uint8_t *buf, uint16_t len);
void display_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb);
void display_port_rx_byte_isr(uint8_t byte);
void display_port_poll_rx(void);

void ibus_port_init(void);
void ibus_port_set_rx_handler(legacy_uart_rx_byte_cb_t cb);
void ibus_port_rx_byte_isr(uint8_t byte);
void ibus_port_poll_rx(void);

void ultrasonic_port_init(void);
uint16_t ultrasonic_port_write(const uint8_t *buf, uint16_t len);
uint16_t ultrasonic_port_read(uint8_t *buf, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* LEGACY_UART_PORT_H */
