#include "uart_task.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_uart_driver.h"
#include "host_cmd_defs.h"
#include "host_control_cmd.h"
#include "host_protocol.h"
#include "telemetry_service.h"
#include "uart_stack.h"
#include "system_cfg.h"

#include "FreeRTOS.h"
#include "task.h"

#if (UART_TASK_COMMAND_BYTE_ORDER != UART_TASK_BYTE_ORDER_LITTLE) && \
    (UART_TASK_COMMAND_BYTE_ORDER != UART_TASK_BYTE_ORDER_BIG)
#error "UART_TASK_COMMAND_BYTE_ORDER must be UART_TASK_BYTE_ORDER_LITTLE or UART_TASK_BYTE_ORDER_BIG"
#endif

#define UART_TASK_SERVICE_PERIOD_MS        10u
#define UART_TASK_TELEMETRY_PERIOD_MS      10u
#define UART_TASK_MAX_LEFT_WHEEL_ANGULAR_MRAD_S   ((int32_t)(1000 * 23))
#define UART_TASK_MAX_RIGHT_WHEEL_ANGULAR_MRAD_S  ((int32_t)(1000 * 23))
#define UART_TASK_SESSION_TIMEOUT_MS       1000u
#define UART_TASK_DRIVE_CMD_TIMEOUT_MS     1000u
#define UART_TASK_LIFT_CMD_TIMEOUT_MS      1000u
#define UART_TASK_RX_PERIOD_MS             10u
#define UART_TASK_RX_CHUNK_MAX             256u

typedef struct
{
    bool started;
    TickType_t last_frame_tick;
    uint8_t session_seen;
    TickType_t last_drive_cmd_tick;
    uint8_t drive_cmd_seen;
    TickType_t last_lift_cmd_tick;
    uint8_t lift_cmd_seen;
    TickType_t last_telemetry_tick;
    uint8_t tx_seq;
    uint16_t latched_status_word;
} uart_task_ctx_t;

static uart_task_ctx_t s_uart_task;
 host_control_cmd_t s_host_control_cmd = {0};

typedef struct
{
    uint8_t has_drive_cmd;
    int32_t left_wheel_angular_mrad_s;
    int32_t right_wheel_angular_mrad_s;
    uint8_t has_lift_target_mm;
    int32_t lift_target_mm;
    uint8_t has_power_policy;
    uint8_t motor_reset_mask;
    host_motor_power_cmd_t right_motor_power_cmd;
    host_motor_power_cmd_t left_motor_power_cmd;
    host_motor_power_cmd_t joint_motor_power_cmd;
} host_control_cmd_update_t;

static inline uint8_t uart_task_session_timed_out(TickType_t now,
                                                  TickType_t last_frame_tick,
                                                  uint8_t session_seen)
{
    if (session_seen == 0u) {
        return 0u;
    }

    return ((now - last_frame_tick) > pdMS_TO_TICKS(UART_TASK_SESSION_TIMEOUT_MS)) ? 1u : 0u;
}

static void uart_task_host_session_reset_locked(void)
{
    uint32_t drive_cmd_seq = s_host_control_cmd.drive_cmd_seq;
    uint32_t lift_cmd_seq = s_host_control_cmd.lift_cmd_seq;

    /* Timeout clears the current data plane, but preserves command epochs so
     * SUB_SLAVE still requires a post-reentry command before motion resumes. */
    memset(&s_host_control_cmd, 0, sizeof(s_host_control_cmd));
    s_host_control_cmd.drive_cmd_seq = drive_cmd_seq;
    s_host_control_cmd.lift_cmd_seq = lift_cmd_seq;
    s_uart_task.latched_status_word = 0u;
    s_uart_task.last_frame_tick = 0u;
    s_uart_task.session_seen = 0u;
    s_uart_task.last_drive_cmd_tick = 0u;
    s_uart_task.drive_cmd_seen = 0u;
    s_uart_task.last_lift_cmd_tick = 0u;
    s_uart_task.lift_cmd_seen = 0u;
}

void host_control_cmd_snapshot(host_control_cmd_t *out)
{
    if (out == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_host_control_cmd;
    taskEXIT_CRITICAL();
}

static void host_control_cmd_apply_update(const host_control_cmd_update_t *update)
{
    if (update == NULL) {
        return;
    }

    /* Communication layer owns the only write entrance to the host mailbox so
     * control/motor tasks can snapshot a coherent command set under one lock. */
    taskENTER_CRITICAL();

    if (update->has_drive_cmd) {
        s_host_control_cmd.left_wheel_angular_mrad_s = update->left_wheel_angular_mrad_s;
        s_host_control_cmd.right_wheel_angular_mrad_s = update->right_wheel_angular_mrad_s;
        ++s_host_control_cmd.drive_cmd_seq;
    }
    if (update->has_lift_target_mm) {
        s_host_control_cmd.z_lift_mm = update->lift_target_mm;
        ++s_host_control_cmd.lift_cmd_seq;
    }
    if (update->has_power_policy) {
        s_host_control_cmd.motor_reset_mask = update->motor_reset_mask;
        s_host_control_cmd.right_motor_power_cmd = update->right_motor_power_cmd;
        s_host_control_cmd.left_motor_power_cmd = update->left_motor_power_cmd;
        s_host_control_cmd.joint_motor_power_cmd = update->joint_motor_power_cmd;
    }

    taskEXIT_CRITICAL();
}

static bool uart_task_tlv_next(const uint8_t *payload,
                               uint16_t len,
                               uint16_t *offset,
                               uint8_t *tag,
                               const uint8_t **value,
                               uint8_t *value_len)
{
    /* Advance strictly by TLV length so unknown tags can be skipped without
     * desynchronizing the rest of the frame. */
    if (payload == NULL || offset == NULL || tag == NULL || value == NULL || value_len == NULL) {
        return false;
    }
    if ((uint16_t)(*offset + 2u) > len) {
        return false;
    }

    *tag = payload[*offset];
    *value_len = payload[(uint16_t)(*offset + 1u)];
    if ((uint16_t)(*offset + 2u + *value_len) > len) {
        return false;
    }

    *value = &payload[(uint16_t)(*offset + 2u)];
    *offset = (uint16_t)(*offset + 2u + *value_len);
    return true;
}

static inline int32_t uart_task_rd_le_s32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

static inline uint16_t uart_task_rd_le_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

#if (UART_TASK_COMMAND_BYTE_ORDER == UART_TASK_BYTE_ORDER_BIG)
static inline int32_t uart_task_rd_cmd_raw_s32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[3] |
                     ((uint32_t)data[2] << 8) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[0] << 24));
}

static inline uint16_t uart_task_rd_cmd_raw_u16(const uint8_t *data)
{
    return (uint16_t)data[1] | ((uint16_t)data[0] << 8);
}
#else
static inline int32_t uart_task_rd_cmd_raw_s32(const uint8_t *data)
{
    return uart_task_rd_le_s32(data);
}

static inline uint16_t uart_task_rd_cmd_raw_u16(const uint8_t *data)
{
    return uart_task_rd_le_u16(data);
}
#endif

static int32_t uart_task_rd_cmd_s32(const uint8_t *data)
{
    return uart_task_rd_cmd_raw_s32(data);
}

static bool uart_task_try_rd_cmd_s32(const uint8_t *data,
                                     int32_t min_value,
                                     int32_t max_value,
                                     int32_t *out)
{
    int32_t value;

    if ((data == NULL) || (out == NULL))
    {
        return false;
    }

    value = uart_task_rd_cmd_s32(data);
    if ((value < min_value) || (value > max_value))
    {
        return false;
    }

    *out = value;
    return true;
}

static bool uart_task_rd_cmd_s32_clamp(const uint8_t *data,
                                       int32_t min_value,
                                       int32_t max_value,
                                       int32_t *out)
{
    int32_t value;

    if ((data == NULL) || (out == NULL) || (min_value > max_value))
    {
        return false;
    }

    value = uart_task_rd_cmd_s32(data);
    if (value < min_value)
    {
        value = min_value;
    }
    else if (value > max_value)
    {
        value = max_value;
    }

    *out = value;
    return true;
}

static uint16_t uart_task_rd_cmd_u16(const uint8_t *data)
{
    return uart_task_rd_cmd_raw_u16(data);
}

static inline uint16_t uart_task_status_word_apply_mask(uint16_t current,
                                                        uint16_t mask,
                                                        uint16_t value)
{
    return (uint16_t)((current & (uint16_t)(~mask)) | (value & mask));
}

static inline host_motor_power_cmd_t uart_task_decode_motor_power_cmd(uint16_t status_word,
                                                                      uint8_t shift)
{
    uint8_t raw_cmd = (uint8_t)((status_word >> shift) & 0x03u);

    if (raw_cmd == 1u) {
        return HOST_MOTOR_POWER_CMD_ENABLE;
    }
    if (raw_cmd == 2u) {
        return HOST_MOTOR_POWER_CMD_DISABLE;
    }

    return HOST_MOTOR_POWER_CMD_NONE;
}

static inline uint8_t uart_task_decode_motor_reset_mask(uint16_t status_word)
{
    uint8_t reset_mask = 0u;

    if (((status_word >> HOST_STATUS_RIGHT_RESET_BIT) & 0x01u) != 0u) {
        reset_mask |= ROBOT_RESET_RIGHT_BIT;
    }
    if (((status_word >> HOST_STATUS_LEFT_RESET_BIT) & 0x01u) != 0u) {
        reset_mask |= ROBOT_RESET_LEFT_BIT;
    }
    if (((status_word >> HOST_STATUS_JOINT_RESET_BIT) & 0x01u) != 0u) {
        reset_mask |= ROBOT_RESET_JOINT_BIT;
    }

    return reset_mask;
}

static void uart_task_session_touch(void)
{
    TickType_t now = xTaskGetTickCount();

    taskENTER_CRITICAL();
    if (uart_task_session_timed_out(now,
                                    s_uart_task.last_frame_tick,
                                    s_uart_task.session_seen) != 0u) {
        uart_task_host_session_reset_locked();
    }
    s_uart_task.last_frame_tick = now;
    s_uart_task.session_seen = 1u;
    taskEXIT_CRITICAL();
}

static void uart_task_drive_cmd_touch(void)
{
    taskENTER_CRITICAL();
    s_uart_task.last_drive_cmd_tick = xTaskGetTickCount();
    s_uart_task.drive_cmd_seen = 1u;
    taskEXIT_CRITICAL();
}

static void uart_task_lift_cmd_touch(void)
{
    taskENTER_CRITICAL();
    s_uart_task.last_lift_cmd_tick = xTaskGetTickCount();
    s_uart_task.lift_cmd_seen = 1u;
    taskEXIT_CRITICAL();
}

static void uart_task_build_telemetry_tlv(uart_builder_t *builder)
{
    telemetry_service_snapshot_t telemetry = {0};

    if (builder == NULL) {
        return;
    }

    /* Telemetry is built from execution-side snapshots only. Optional fields are
     * emitted only when a real producer marks them valid. */
    telemetry_service_snapshot(&telemetry);

    (void)uart_builder_add_s32(builder,
                               UART_TASK_TAG_LEFT_WHEEL_ANGULAR_MRAD_S,
                               telemetry.left_wheel_angular_mrad_s);
    (void)uart_builder_add_s32(builder,
                               UART_TASK_TAG_RIGHT_WHEEL_ANGULAR_MRAD_S,
                               telemetry.right_wheel_angular_mrad_s);
    (void)uart_builder_add_s32(builder,
                               UART_TASK_TAG_Z_LIFT_MM,
                               telemetry.z_lift_mm);
    if (telemetry.bucket_volume_valid != 0u) {
        (void)uart_builder_add_u16(builder,
                                   UART_TASK_TAG_BUCKET_VOLUME_ML,
                                   telemetry.bucket_volume_ml);
    }
    (void)uart_builder_add_u16(builder,
                               UART_TASK_TAG_STATUS_WORD,
                               telemetry.status_word);
    (void)uart_builder_add_u16(builder,
                               UART_TASK_TAG_HEALTH_WORD,
                               telemetry.health_word);
    (void)uart_builder_add_u16(builder,
                               UART_TASK_TAG_ALARM_INFO,
                               telemetry.alarm_info);
    (void)uart_builder_add_u16(builder,
                               UART_TASK_TAG_BATT_SOC_X100,
                               telemetry.batt_soc_x100);
}

static uint16_t uart_task_build_telemetry_frame(uart_task_ctx_t *ctx,
                                                uint8_t *out,
                                                uint16_t maxlen)
{
    uart_builder_t builder;
    uint16_t out_len = 0u;
    TickType_t now = xTaskGetTickCount();
    uint8_t seq = 0u;

    if ((ctx == NULL) || (out == NULL) || (maxlen == 0u))
    {
        return 0u;
    }

    /* Throttle telemetry to the configured 100Hz budget and allocate tx_seq in
     * the same critical section so periodic sends stay monotonic. */
    taskENTER_CRITICAL();
    if ((ctx->last_telemetry_tick != 0u) &&
        ((now - ctx->last_telemetry_tick) < pdMS_TO_TICKS(UART_TASK_TELEMETRY_PERIOD_MS)))
    {
        taskEXIT_CRITICAL();
        return 0u;
    }
    ctx->last_telemetry_tick = now;
    seq = ctx->tx_seq++;
    taskEXIT_CRITICAL();

    uart_builder_begin(&builder, out, maxlen, seq, UART_TASK_FRAME_TYPE_TELEMETRY);
    uart_task_build_telemetry_tlv(&builder);
    if (!uart_builder_end(&builder, &out_len)) {
        return 0u;
    }

    return out_len;
}

static uint16_t uart_task_build_tx(uint8_t uart_id, uint8_t *out, uint16_t maxlen, void *user_ctx)
{
    uart_task_ctx_t *ctx = (uart_task_ctx_t *)user_ctx;

    (void)uart_id;

    if (ctx == NULL || out == NULL || maxlen == 0u) {
        return 0u;
    }

    return uart_task_build_telemetry_frame(ctx, out, maxlen);
}

static void uart_task_apply_command(const uart_stack_frame_t *frame)
{
    host_control_cmd_update_t update = {0};
    uint16_t offset = 0u;
    bool invalid_drive = false;
    bool invalid_lift = false;
    bool invalid_power_policy = false;
    bool drive_cmd_ready = false;
    bool lift_cmd_ready = false;
    bool power_policy_ready = false;
    bool has_left_wheel_angular = false;
    bool has_right_wheel_angular = false;
    bool has_z_lift = false;
    bool has_mask = false;
    bool has_value = false;
    int32_t left_wheel_angular_mrad_s = 0;
    int32_t right_wheel_angular_mrad_s = 0;
    int32_t z_lift_mm = 0;
    uint16_t mask = 0u;
    uint16_t value = 0u;
    uint16_t latched_status_word = 0u;
    uint8_t tag = 0u;
    uint8_t value_len = 0u;
    const uint8_t *tlv_value = NULL;

    if (frame == NULL || frame->payload == NULL) {
        return;
    }

    /* Parse drive, lift and power-policy domains independently so one invalid
     * TLV set does not silently corrupt another domain in the same frame. */
    while (uart_task_tlv_next(frame->payload, frame->len, &offset, &tag, &tlv_value, &value_len))
    {
        if (tag == UART_TASK_TAG_LEFT_WHEEL_ANGULAR_MRAD_S) {
            if (value_len != 4u) {
                invalid_drive = true;
                continue;
            }
            has_left_wheel_angular = true;
            if (!uart_task_try_rd_cmd_s32(tlv_value,
                                          -UART_TASK_MAX_LEFT_WHEEL_ANGULAR_MRAD_S,
                                          UART_TASK_MAX_LEFT_WHEEL_ANGULAR_MRAD_S,
                                          &left_wheel_angular_mrad_s)) {
                invalid_drive = true;
            }
        } else if (tag == UART_TASK_TAG_RIGHT_WHEEL_ANGULAR_MRAD_S) {
            if (value_len != 4u) {
                invalid_drive = true;
                continue;
            }
            has_right_wheel_angular = true;
            if (!uart_task_try_rd_cmd_s32(tlv_value,
                                          -UART_TASK_MAX_RIGHT_WHEEL_ANGULAR_MRAD_S,
                                          UART_TASK_MAX_RIGHT_WHEEL_ANGULAR_MRAD_S,
                                          &right_wheel_angular_mrad_s)) {
                invalid_drive = true;
            }
        } else if (tag == UART_TASK_TAG_Z_LIFT_MM) {
            if (value_len != 4u) {
                invalid_lift = true;
                continue;
            }
            has_z_lift = true;
            /* Lift commands saturate into the configured travel range so the
             * host can overshoot slightly without losing the whole command. */
            if (!uart_task_rd_cmd_s32_clamp(tlv_value,
                                            0,
                                            (int32_t)SYSTEM_CFG_JOINT_MAX_TRAVEL_MM,
                                            &z_lift_mm)) {
                invalid_lift = true;
            }
        } else if (tag == UART_TASK_TAG_STATUS_MASK) {
            if (value_len != 2u) {
                invalid_power_policy = true;
                continue;
            }
            has_mask = true;
            mask = uart_task_rd_cmd_u16(tlv_value);
        } else if (tag == UART_TASK_TAG_STATUS_VALUE) {
            if (value_len != 2u) {
                invalid_power_policy = true;
                continue;
            }
            has_value = true;
            value = uart_task_rd_cmd_u16(tlv_value);
        }
    }

    if (has_left_wheel_angular != has_right_wheel_angular)
    {
        /* Differential drive velocity is atomic: left/right tags must arrive together. */
        invalid_drive = true;
    }
    if (has_mask != has_value)
    {
        /* status_mask and status_value form one latched update and must be paired. */
        invalid_power_policy = true;
    }

    drive_cmd_ready = (!invalid_drive) && has_left_wheel_angular && has_right_wheel_angular;
    lift_cmd_ready = (!invalid_lift) && has_z_lift;
    power_policy_ready = (!invalid_power_policy) && has_mask && has_value;

    if (!(drive_cmd_ready || lift_cmd_ready || power_policy_ready)) {
        return;
    }

    if (drive_cmd_ready) {
        uart_task_drive_cmd_touch();
        update.has_drive_cmd = 1u;
        update.left_wheel_angular_mrad_s = left_wheel_angular_mrad_s;
        update.right_wheel_angular_mrad_s = right_wheel_angular_mrad_s;
    }
    if (lift_cmd_ready) {
        uart_task_lift_cmd_touch();
        update.has_lift_target_mm = 1u;
        update.lift_target_mm = z_lift_mm;
    }
    if (power_policy_ready) {
        taskENTER_CRITICAL();
        /* Masked writes let the host change one policy bit without resending the
         * whole status word, while preserving the rest of the latched policy. */
        s_uart_task.latched_status_word =
            uart_task_status_word_apply_mask(s_uart_task.latched_status_word, mask, value);
        latched_status_word = s_uart_task.latched_status_word;
        taskEXIT_CRITICAL();

        /* Translate policy bits into mailbox intent here; the motor task still
         * decides whether the current mode/session allows them to execute. */
        update.has_power_policy = 1u;
        update.motor_reset_mask = uart_task_decode_motor_reset_mask(latched_status_word);
        update.right_motor_power_cmd = uart_task_decode_motor_power_cmd(latched_status_word,
                                                                        HOST_STATUS_RIGHT_ENABLE_SHIFT);
        update.left_motor_power_cmd = uart_task_decode_motor_power_cmd(latched_status_word,
                                                                       HOST_STATUS_LEFT_ENABLE_SHIFT);
        update.joint_motor_power_cmd = uart_task_decode_motor_power_cmd(latched_status_word,
                                                                        HOST_STATUS_JOINT_ENABLE_SHIFT);
    }

    host_control_cmd_apply_update(&update);
}

static void uart_task_on_frame(uint8_t uart_id, const uart_stack_frame_t *frame, void *user_ctx)
{
    (void)uart_id;
    (void)user_ctx;

    if (frame == NULL) {
        return;
    }

    /* Any valid frame keeps the host session alive, but only command frames are
     * allowed to mutate the host mailbox. */
    uart_task_session_touch();

    if (frame->type != UART_TASK_FRAME_TYPE_COMMAND) {
        return;
    }

    uart_task_apply_command(frame);
}

bool uart_task_start(void)
{
    uart_stack_cfg_t stack_cfg;

    if (s_uart_task.started) {
        return true;
    }

    memset(&s_uart_task, 0, sizeof(s_uart_task));
    memset(&stack_cfg, 0, sizeof(stack_cfg));

    stack_cfg.uart_id = DEV_UART1;
    stack_cfg.tx_period_ms = UART_TASK_SERVICE_PERIOD_MS;
    stack_cfg.rx_period_ms = UART_TASK_RX_PERIOD_MS;
    stack_cfg.rx_chunk_max = UART_TASK_RX_CHUNK_MAX;
    stack_cfg.build_tx = uart_task_build_tx;
    stack_cfg.on_frame = uart_task_on_frame;
    stack_cfg.user_ctx = &s_uart_task;

    if (!uart_stack_start(&stack_cfg)) {
        return false;
    }

    s_uart_task.started = true;
    return true;
}

void uart_task_diag_maintenance(void)
{
    TickType_t now = xTaskGetTickCount();

    /* Age UART ownership and telemetry/CAN diagnostics together so communication
     * health is maintained from one periodic maintenance point. */
    taskENTER_CRITICAL();
    if (uart_task_session_timed_out(now,
                                    s_uart_task.last_frame_tick,
                                    s_uart_task.session_seen) != 0u) {
        uart_task_host_session_reset_locked();
    }
    taskEXIT_CRITICAL();

    telemetry_service_diag_maintenance();
}

static bool uart_task_activity_is_alive(TickType_t last_tick,
                                        uint8_t seen,
                                        uint32_t timeout_ms)
{
    TickType_t now;

    if (seen == 0u) {
        return false;
    }

    now = xTaskGetTickCount();
    return ((now - last_tick) <= pdMS_TO_TICKS(timeout_ms));
}

bool uart_session_is_alive(void)
{
    TickType_t last_frame_tick;
    uint8_t session_seen;

    taskENTER_CRITICAL();
    last_frame_tick = s_uart_task.last_frame_tick;
    session_seen = s_uart_task.session_seen;
    taskEXIT_CRITICAL();

    return uart_task_activity_is_alive(last_frame_tick,
                                       session_seen,
                                       UART_TASK_SESSION_TIMEOUT_MS);
}

bool uart_drive_cmd_is_fresh(void)
{
    TickType_t last_drive_cmd_tick;
    uint8_t drive_cmd_seen;

    taskENTER_CRITICAL();
    last_drive_cmd_tick = s_uart_task.last_drive_cmd_tick;
    drive_cmd_seen = s_uart_task.drive_cmd_seen;
    taskEXIT_CRITICAL();

    return uart_task_activity_is_alive(last_drive_cmd_tick,
                                       drive_cmd_seen,
                                       UART_TASK_DRIVE_CMD_TIMEOUT_MS);
}

bool uart_lift_cmd_is_fresh(void)
{
    TickType_t last_lift_cmd_tick;
    uint8_t lift_cmd_seen;

    taskENTER_CRITICAL();
    last_lift_cmd_tick = s_uart_task.last_lift_cmd_tick;
    lift_cmd_seen = s_uart_task.lift_cmd_seen;
    taskEXIT_CRITICAL();

    return uart_task_activity_is_alive(last_lift_cmd_tick,
                                       lift_cmd_seen,
                                       UART_TASK_LIFT_CMD_TIMEOUT_MS);
}
