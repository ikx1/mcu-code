/**
 * @file uart.h
 */
#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

#define MODBUS_USART3

#ifdef MODBUS_USART3
#define MODBUS_USARTx                   USART3
#define MODBUS_USART_CLK                RCC_APB1Periph_USART3
#define MODBUS_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define MODBUS_USART_BAUDRATE           19200u
#define MODBUS_USART_GPIO_CLK           RCC_APB2Periph_GPIOB
#define MODBUS_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
#define MODBUS_USART_TX_GPIO_PORT       GPIOB
#define MODBUS_USART_TX_GPIO_PIN        GPIO_Pin_10
#define MODBUS_USART_RX_GPIO_PORT       GPIOB
#define MODBUS_USART_RX_GPIO_PIN        GPIO_Pin_11
#define MODBUS_USART_IRQ                USART3_IRQn
#define MODBUS_USART_IRQHandler         USART3_IRQHandler
#endif

#define MODBUS_ULTRASONIC_USARTx                USART2
#define MODBUS_ULTRASONIC_USART_CLK             RCC_APB1Periph_USART2
#define MODBUS_ULTRASONIC_USART_APBxClkCmd      RCC_APB1PeriphClockCmd
#define MODBUS_ULTRASONIC_BAUDRATE              115200u
#define MODBUS_ULTRASONIC_USART_GPIO_CLK        RCC_APB2Periph_GPIOA
#define MODBUS_ULTRASONIC_USART_GPIO_APBxClkCmd RCC_APB2PeriphClockCmd
#define MODBUS_ULTRASONIC_USART_TX_GPIO_PORT    GPIOA
#define MODBUS_ULTRASONIC_USART_TX_GPIO_PIN     GPIO_Pin_2
#define MODBUS_ULTRASONIC_USART_RX_GPIO_PORT    GPIOA
#define MODBUS_ULTRASONIC_USART_RX_GPIO_PIN     GPIO_Pin_3
#define MODBUS_ULTRASONIC_USART_IRQ             USART2_IRQn
#define MODBUS_ULTRASONIC_USART_IRQHandler      USART2_IRQHandler

void uart1_init(void);
void uart1_dmatx_config(uint8_t *mem_addr, uint32_t mem_size);
void uart1_dmarx_config(uint8_t *mem_addr, uint32_t mem_size);
uint16_t uart1_get_dmarx_buf_remain_size(void);

void uart2_init(void);
void modbus_ultrasonic_sendbuf(uint8_t *str, uint8_t len);

void uart3_init(void);
void Modbus_SendStr(uint8_t data);
void Modbus_SendBuf(uint8_t *str, uint8_t len);

void uart4_init(void);
void uart4_send_str(uint8_t *send_str, uint8_t num);

void uart5_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
