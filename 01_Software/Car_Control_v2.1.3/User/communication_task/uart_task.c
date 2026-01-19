#include "uart_task.h"
#include "uart_proto_newfmt.h"

#include "bsp_battery_handler.h"
#include "bsp_gpio_driver.h"
#include "bsp_ibus_handler.h"
#include "bsp_motor_handler.h"

#include "control_task.h"
#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/********************************** Defines **********************************/
#define MAX_X_SPEED    ((int32_t)(1000 * 0.5f))
#define MAX_Z_SPEED    ((int32_t)(1000 * 0.4f))
#define MAX_JOINT_POS  ((int32_t)(250))

#define UART_LINK_TIMEOUT_MS  2000u

/********************************** Local helpers **********************************/
static volatile TickType_t s_uart_last_rx_tick = 0;
static volatile uint8_t s_uart_link_seen = 0;

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

static bool tlv_next(const uint8_t *p, uint16_t len, uint16_t *off,
                     uint8_t *tag, const uint8_t **v, uint8_t *vlen)
{
    if (*off + 2 > len) return false;
    *tag  = p[*off + 0];
    *vlen = p[*off + 1];
    if (*off + 2 + *vlen > len) return false;
    *v = &p[*off + 2];
    *off = (uint16_t)(*off + 2 + *vlen);
    return true;
}

static inline int32_t rd_le_s32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                    ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) |
                    ((uint32_t)p[3] << 24));
}
static inline uint16_t rd_le_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void uart_link_touch(void)
{
	taskENTER_CRITICAL();
	s_uart_last_rx_tick = xTaskGetTickCount();
	s_uart_link_seen = 1u;
	taskEXIT_CRITICAL();
}

/********************************** Protocol context **********************************/
typedef struct {
    uart_parser_t parser;
    uint8_t tx_seq;

    /* ACK pending (shared TX/RX tasks) */
    uint8_t ack_pending;
    uint8_t ack_seq;
    uint8_t ack_code;
} uart_proto_ctx_t;

static uart_proto_ctx_t g_ctx;

void uart_task_protocol_init(void)
{
    memset(&g_ctx, 0, sizeof(g_ctx));
    uart_parser_init(&g_ctx.parser);
    g_ctx.tx_seq = 0;

	taskENTER_CRITICAL();
	s_uart_last_rx_tick = 0;
	s_uart_link_seen = 0;
	taskEXIT_CRITICAL();
}

bool uart_link_is_alive(void)
{
	TickType_t now = xTaskGetTickCount();
	TickType_t last;
	uint8_t seen;

	taskENTER_CRITICAL();
	last = s_uart_last_rx_tick;
	seen = s_uart_link_seen;
	taskEXIT_CRITICAL();

	if (!seen)
	{
		return false;
	}
	if ((now - last) > pdMS_TO_TICKS(UART_LINK_TIMEOUT_MS))
	{
		return false;
	}
	return true;
}

/********************************** Telemetry builder **********************************/
static void build_telemetry_tlv(uart_builder_t *b)
{
    const car_info_t* speed_info = get_car_info_p();
    const BATTERY_INFO* bi = BatteryHandler_GetBatteryInfo();
    motor_driver_t *m = get_motor_drivers_p();
    if(NULL == speed_info || NULL == bi || NULL == m)
        return;

    uint8_t right_motor_enable = (m[0].info->state == MOTOR_STATE_READING) ? 1u : 0u;
    uint8_t left_motor_enable = (m[1].info->state == MOTOR_STATE_READING) ? 1u : 0u;
//    uint8_t slide_table_enable = (m[2].info->state == JOINT_ENABLE) ? 1u : 0u;

    uint8_t right_motor_alarm = (m[0].info->state == MOTOR_STATE_ALARM) ? 1u : 0u;
    uint8_t left_motor_alarm = (m[1].info->state == MOTOR_STATE_ALARM) ? 1u : 0u;
//    uint8_t slide_table_alarm = (m[2].info->state == JOINT_ALARM) ? 1u : 0u;

    /* health_word： */
    uint16_t health_word = 0;
    health_word |= ((input_driver_get_status()->emergency_flag & 0x01) << 0);
    health_word |= ((right_motor_enable & 0x01) << 1);
    health_word |= ((right_motor_alarm & 0x01) << 2);
    health_word |= ((left_motor_enable & 0x01) << 3);
    health_word |= ((left_motor_alarm & 0x01) << 4);
    health_word |= ((0 & 0x01) << 5);
    health_word |= ((0 & 0x01) << 6);
    health_word |= ((0 & 0x01) << 7);
    health_word |= ((bi->comm_fault & 0x01) << 8);
    health_word |= ((get_ibus_data_p()->connected & 0x01) << 9);
    health_word |= ((0 & 0x01) << 10);
    health_word |= ((0 & 0x01) << 11);
    health_word |= ((bi->charge_status & 0x01) << 12);
    health_word |= ((0 & 0x01) << 13);
    health_word |= ((0 & 0x01) << 14);
    health_word |= ((0 & 0x01) << 15);

    /* v/w（单位是 mm/s 和 mrad/s） */
    (void)uart_builder_add_s32(b, TAG_V_LINEAR_MM_S,    (int32_t)speed_info->global_speed);
    (void)uart_builder_add_s32(b, TAG_W_ANGULAR_MRAD_S, (int32_t)speed_info->global_roation);
    (void)uart_builder_add_s32(b, TAG_Z_LIFT_01MM, 0);

    (void)uart_builder_add_u16(b, TAG_HEALTH_WORD, health_word);

    /* 电量：remain_capacity 若本来就是 0..10000 表示 0..100.00% */
    (void)uart_builder_add_u16(b, TAG_BATT_SOC_X100, (uint16_t)bi->remain_capacity);
}

/********************************** Command apply **********************************/
static void apply_command_from_frame(const uart_frame_t *f)
{
    ROBOT_INFO* car = get_robot_p();

    bool has_v = false, has_w = false, has_joint_pos = false;
    int32_t v_mm_s = 0, w_mrad_s = 0, joint_pos = 0;

    bool has_mask = false, has_val = false;
    uint16_t mask = 0, value = 0;

    uint16_t off = 0;
    uint8_t tag, vlen;
    const uint8_t *v;

	uart_link_touch();
	if (car == NULL)
	{
		return;
	}

    while (tlv_next(f->payload, f->len, &off, &tag, &v, &vlen))
    {
        if (tag == TAG_V_LINEAR_MM_S && vlen == 4) { has_v = true; v_mm_s = rd_le_s32(v); }
        else if (tag == TAG_W_ANGULAR_MRAD_S && vlen == 4) { has_w = true; w_mrad_s = rd_le_s32(v); }
        else if (tag == TAG_Z_LIFT_01MM && vlen == 4) { has_joint_pos = true; joint_pos = rd_le_s32(v); }
        else if (tag == TAG_STATUS_MASK && vlen == 2) { has_mask = true; mask = rd_le_u16(v); }
        else if (tag == TAG_STATUS_VALUE && vlen == 2) { has_val = true; value = rd_le_u16(v); }
        else { /* unknown */ }
    }

	bool has_status = (has_mask && has_val);

    if (has_v) {
        v_mm_s = clamp_i32(v_mm_s, -MAX_X_SPEED, MAX_X_SPEED);
    }
    if (has_w) {
        w_mrad_s = clamp_i32(w_mrad_s, -MAX_Z_SPEED, MAX_Z_SPEED);
    }
    if (has_joint_pos) {
        joint_pos = clamp_i32(joint_pos, -MAX_JOINT_POS, MAX_JOINT_POS);
    }

	if (has_v || has_w || has_joint_pos || has_status)
	{
		taskENTER_CRITICAL();
		if (has_v) {
			car->glob_line_speed = v_mm_s;
		}
		if (has_w) {
			car->glob_rota_speed = w_mrad_s;
		}
		if (has_joint_pos) {
			car->glob_joint_pos = joint_pos;
		}
		if (has_status) {
			/* requires car->status_word field */
			car->status_word = (car->status_word & (uint16_t)(~mask)) | (value & mask);
		}
		taskEXIT_CRITICAL();
	}

//    if (ack_req) {
//        taskENTER_CRITICAL();
//        g_ctx.ack_pending = 1;
//        g_ctx.ack_seq = f->seq;
//        g_ctx.ack_code = 0; /* OK */
//        taskEXIT_CRITICAL();
//    }
}

/********************************** uart_rtos callbacks **********************************/
uint16_t app_uart_build_tx(uint8_t uart_id, uint8_t *out, uint16_t maxlen, void *user_ctx)
{
    (void)uart_id;
    (void)user_ctx;

    uart_builder_t b;
    uint16_t out_len = 0;

    // /* ACK 优先（临界区只做快照/清除） */
    // uint8_t pending, ack_seq, ack_code;
    // taskENTER_CRITICAL();
    // pending = g_ctx.ack_pending;
    // ack_seq = g_ctx.ack_seq;
    // ack_code = g_ctx.ack_code;
    // /* 先不清，构建成功后再清，避免丢ACK */
    // taskEXIT_CRITICAL();

    // if (pending)
    // {
        // uart_builder_begin(&b, out, maxlen, ack_seq, UART_TYPE_ACK);
        // if (uart_builder_end(&b, &out_len)) {
        //     taskENTER_CRITICAL();
        //     /* 只清除“同一 seq”的 pending，避免与 RX 更新冲突 */
        //     if (g_ctx.ack_pending && g_ctx.ack_seq == ack_seq) {
        //         g_ctx.ack_pending = 0;
        //     }
        //     taskEXIT_CRITICAL();
        //     return out_len;
        // }
        // return 0;
    // }

    /* Telemetry */
    uart_builder_begin(&b, out, maxlen, g_ctx.tx_seq++, UART_TYPE_TELEMETRY);
    build_telemetry_tlv(&b);
    if (!uart_builder_end(&b, &out_len)) return 0;
    return out_len;
}

void app_uart_on_rx(uint8_t uart_id, const uint8_t *data, uint16_t len, void *user_ctx)
{
    (void)uart_id;
    (void)user_ctx;

    uart_frame_t f;
    for (uint16_t i = 0; i < len; i++)
    {
        if (uart_parser_feed(&g_ctx.parser, data[i], &f))
        {
            if (f.type == UART_TYPE_COMMAND) {
                apply_command_from_frame(&f);
            }
        }
    }
}
