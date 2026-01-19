#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#include <stdint.h>
#include <stdbool.h>

#define H1 0xA0
#define H2 0x0A

/* Types */
#define UART_TYPE_TELEMETRY   0x01
#define UART_TYPE_COMMAND     0x81
#define UART_TYPE_ACK         0x02

/* TLV Tags（按你协议字典） */
#define TAG_V_LINEAR_MM_S     0x01  /* int32 */
#define TAG_W_ANGULAR_MRAD_S  0x02  /* int32 */
#define TAG_Z_LIFT_01MM       0x03  /* int32 */

#define TAG_STATUS_WORD       0x13  /* u16 */
#define TAG_HEALTH_WORD       0x14  /* u16 */

#define TAG_STATUS_MASK       0x11  /* u16 */
#define TAG_STATUS_VALUE      0x12  /* u16 */

#define TAG_BATT_SOC_X100     0x21  /* u16：电量百分比*100（可选，替代float） */

#define TAG_ACK_RESULT        0x30  /* u8: 0=OK, 其它=错误码 */
#define TAG_ACK_REQ           0x40  /* u8: 1=要求ACK（可选） */

uint16_t app_uart_build_tx(uint8_t uart_id, uint8_t *out, uint16_t maxlen, void *user_ctx);
void app_uart_on_rx(uint8_t uart_id, const uint8_t *data, uint16_t len, void *user_ctx);

/* 在 uart_rtos_start 之前调用一次 */
void uart_task_protocol_init(void);
bool uart_link_is_alive(void);

#endif
