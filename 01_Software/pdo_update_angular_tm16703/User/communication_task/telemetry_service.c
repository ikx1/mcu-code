#include "telemetry_service.h"

#include <stdint.h>
#include <string.h>

#include "bsp_battery_handler.h"
#include "bsp_gpio_driver.h"
#include "bsp_ibus_handler.h"
#include "bsp_motor_handler.h"
#include "can_irq_adapter.h"
#include "can_queue.h"
#include "drive_feedback_service.h"
#include "host_status_defs.h"
#include "system_cfg.h"

#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
    uint8_t inited;
    uint32_t last_rx_drop_count;
    uint32_t last_busoff_count;
    uint32_t last_recover_fail_count;
    TickType_t last_change_tick;
    TickType_t last_rx_drop_event_tick;
    TickType_t last_busoff_event_tick;
    TickType_t last_recover_fail_event_tick;
} telemetry_service_can_diag_t;

static telemetry_service_can_diag_t s_telemetry_can_diag = {0};

static uint8_t telemetry_service_joint_enabled_state(motor_state_t state)
{
    return ((state == JOINT_ENABLE) || ((uint16_t)state == 0x0433u)) ? 1u : 0u;
}

static uint8_t telemetry_service_motor_state_match(uint8_t index,
                                                   motor_state_t state)
{
    motor_info_t info = {0};

    if (index >= MOTOR_NUM)
    {
        return 0u;
    }

    if (motor_info_snapshot((motor_index_t)index, &info) != 0u)
    {
        return 0u;
    }

    return (info.state == state) ? 1u : 0u;
}

static uint8_t telemetry_service_motor_enabled(motor_index_t index)
{
    motor_info_t info = {0};

    switch (index)
    {
        case MOTOR_INDEX_RIGHT:
        case MOTOR_INDEX_LEFT:
            return telemetry_service_motor_state_match((uint8_t)index, MOTOR_STATE_READING);

        case MOTOR_INDEX_JOINT:
            if (motor_info_snapshot(index, &info) != 0u)
            {
                return 0u;
            }
            return telemetry_service_joint_enabled_state(info.state);

        default:
            return 0u;
    }
}

static uint8_t telemetry_service_motor_alarm(motor_index_t index)
{
    switch (index)
    {
        case MOTOR_INDEX_RIGHT:
        case MOTOR_INDEX_LEFT:
            return telemetry_service_motor_state_match((uint8_t)index, MOTOR_STATE_ALARM);

        case MOTOR_INDEX_JOINT:
            return telemetry_service_motor_state_match((uint8_t)index, JOINT_ALARM);

        default:
            return 0u;
    }
}

static int32_t telemetry_service_joint_position_mm(uint8_t index)
{
    motor_info_t info = {0};

    if (index >= MOTOR_NUM)
    {
        return 0;
    }

    if (motor_info_snapshot((motor_index_t)index, &info) != 0u)
    {
        return 0;
    }

    return (int32_t)(info.location);
}

static uint8_t telemetry_service_motor_alarm_code(motor_index_t index)
{
    motor_info_t info = {0};

    if (motor_info_snapshot(index, &info) != 0u)
    {
        return 0u;
    }

    if (info.state != MOTOR_STATE_ALARM)
    {
        return 0u;
    }

    return (uint8_t)(info.state_err & 0xFFu);
}

static uint16_t telemetry_service_build_status_word(const BATTERY_INFO *battery,
                                                    uint8_t battery_valid)
{
    uint16_t status_word = 0u;
    /* Upload only actual device state here. Unsupported auxiliary actuators stay 0. */

    if (telemetry_service_motor_enabled(MOTOR_INDEX_RIGHT) != 0u)
    {
        status_word |= (uint16_t)(1u << HOST_STATUS_RIGHT_ENABLE_SHIFT);
    }
    if (telemetry_service_motor_enabled(MOTOR_INDEX_LEFT) != 0u)
    {
        status_word |= (uint16_t)(1u << HOST_STATUS_LEFT_ENABLE_SHIFT);
    }
    if (telemetry_service_motor_enabled(MOTOR_INDEX_JOINT) != 0u)
    {
        status_word |= (uint16_t)(1u << HOST_STATUS_JOINT_ENABLE_SHIFT);
    }

    if ((battery_valid != 0u) && ((battery->charge_status & 0x01u) != 0u))
    {
        status_word |= (uint16_t)((uint16_t)1u << HOST_STATUS_CHARGING_ON_BIT);
    }

    return status_word;
}

static uint16_t telemetry_service_build_health_word(const BATTERY_INFO *battery,
                                                    uint8_t battery_valid,
                                                    const IBUS_Handler *ibus,
                                                    uint8_t ibus_valid,
                                                    const input_status_t *input)
{
    uint16_t health_word = 0u;
    /* Health bits must reflect observed hardware/session state, not host intent. */

    if (input != NULL)
    {
        health_word |= (uint16_t)((input->emergency_flag & 0x01u) << 0);
    }

    health_word |= (uint16_t)(telemetry_service_motor_enabled(MOTOR_INDEX_RIGHT) << 1);
    health_word |= (uint16_t)(telemetry_service_motor_alarm(MOTOR_INDEX_RIGHT) << 2);
    health_word |= (uint16_t)(telemetry_service_motor_enabled(MOTOR_INDEX_LEFT) << 3);
    health_word |= (uint16_t)(telemetry_service_motor_alarm(MOTOR_INDEX_LEFT) << 4);
    health_word |= (uint16_t)(telemetry_service_motor_enabled(MOTOR_INDEX_JOINT) << 5);
    health_word |= (uint16_t)(telemetry_service_motor_alarm(MOTOR_INDEX_JOINT) << 6);

    if (battery_valid != 0u)
    {
        health_word |= (uint16_t)((battery->comm_fault & 0x01u) << 8);
        health_word |= (uint16_t)((battery->charge_status & 0x01u) << 12);
    }

    if (ibus_valid != 0u)
    {
        health_word |= (uint16_t)(((ibus->connected ? 1u : 0u) & 0x01u) << 9);
    }

    return health_word;
}

static uint16_t telemetry_service_build_alarm_info(void)
{
    uint8_t right_alarm = telemetry_service_motor_alarm_code(MOTOR_INDEX_RIGHT);
    uint8_t left_alarm = telemetry_service_motor_alarm_code(MOTOR_INDEX_LEFT);

    return (uint16_t)(((uint16_t)left_alarm << 8) | (uint16_t)right_alarm);
}

static void telemetry_service_can_diag_update_state(const can_irq_diag_t *can_diag,
                                                    uint32_t can_rx_drop,
                                                    uint8_t can_rx_depth,
                                                    uint8_t can_tx_pending,
                                                    TickType_t now)
{
    if (can_diag == NULL)
    {
        return;
    }

    /* Keep recent CAN faults as an event window rather than a one-sample value,
     * so telemetry can report bursts that may have already recovered. */
    if (s_telemetry_can_diag.inited == 0u)
    {
        s_telemetry_can_diag.inited = 1u;
        s_telemetry_can_diag.last_rx_drop_count = can_rx_drop;
        s_telemetry_can_diag.last_busoff_count = can_diag->busoff_count;
        s_telemetry_can_diag.last_recover_fail_count = can_diag->recover_fail_count;
        s_telemetry_can_diag.last_change_tick = now;
        return;
    }

    if (can_rx_drop != s_telemetry_can_diag.last_rx_drop_count)
    {
        s_telemetry_can_diag.last_rx_drop_count = can_rx_drop;
        s_telemetry_can_diag.last_rx_drop_event_tick = now;
        s_telemetry_can_diag.last_change_tick = now;
    }

    if (can_diag->busoff_count != s_telemetry_can_diag.last_busoff_count)
    {
        s_telemetry_can_diag.last_busoff_count = can_diag->busoff_count;
        s_telemetry_can_diag.last_busoff_event_tick = now;
        s_telemetry_can_diag.last_change_tick = now;
    }

    if (can_diag->recover_fail_count != s_telemetry_can_diag.last_recover_fail_count)
    {
        s_telemetry_can_diag.last_recover_fail_count = can_diag->recover_fail_count;
        s_telemetry_can_diag.last_recover_fail_event_tick = now;
        s_telemetry_can_diag.last_change_tick = now;
    }

    if ((can_diag->recover_pending != 0u) || (can_rx_depth != 0u) || (can_tx_pending != 0u))
    {
        s_telemetry_can_diag.last_change_tick = now;
    }
}

static uint8_t telemetry_service_can_diag_need_clear(const can_irq_diag_t *can_diag,
                                                     uint8_t can_rx_depth,
                                                     uint8_t can_tx_pending,
                                                     TickType_t now)
{
    if (can_diag == NULL)
    {
        return 0u;
    }

    if (s_telemetry_can_diag.inited == 0u)
    {
        return 0u;
    }

    if (can_diag->recover_pending != 0u)
    {
        return 0u;
    }

    if ((can_rx_depth != 0u) || (can_tx_pending != 0u))
    {
        return 0u;
    }

    if ((now - s_telemetry_can_diag.last_change_tick) <= pdMS_TO_TICKS(CAN_DIAG_COUNTER_AUTO_CLEAR_MS))
    {
        return 0u;
    }

    return 1u;
}

void telemetry_service_diag_maintenance(void)
{
    can_irq_diag_t can_diag = {0};
    TickType_t now = xTaskGetTickCount();
    uint32_t can_rx_drop = can_rx_fifo_drop_count_get();
    uint8_t can_rx_depth = can_rx_fifo_count_get();
    uint8_t can_tx_pending = can_tx_queue_pending_count_get();
    uint8_t need_clear = 0u;

    mcu_can_diag_snapshot(&can_diag);

    taskENTER_CRITICAL();
    /* Clear sticky counters only after the bus has stayed quiet for a full
     * stability window, so historical faults do not remain latched forever. */
    telemetry_service_can_diag_update_state(&can_diag,
                                            can_rx_drop,
                                            can_rx_depth,
                                            can_tx_pending,
                                            now);
    need_clear = telemetry_service_can_diag_need_clear(&can_diag,
                                                       can_rx_depth,
                                                       can_tx_pending,
                                                       now);
    if (need_clear != 0u)
    {
        s_telemetry_can_diag.last_rx_drop_count = 0u;
        s_telemetry_can_diag.last_busoff_count = 0u;
        s_telemetry_can_diag.last_recover_fail_count = 0u;
        s_telemetry_can_diag.last_change_tick = now;
        s_telemetry_can_diag.last_rx_drop_event_tick = 0u;
        s_telemetry_can_diag.last_busoff_event_tick = 0u;
        s_telemetry_can_diag.last_recover_fail_event_tick = 0u;
    }
    taskEXIT_CRITICAL();

    if (need_clear != 0u)
    {
        can_rx_fifo_drop_count_clear();
        mcu_can_diag_reset_counters();
    }
}

void telemetry_service_snapshot(telemetry_service_snapshot_t *out)
{
    drive_feedback_t drive_feedback = {0};
    BATTERY_INFO battery = {0};
    IBUS_Handler ibus = {0};
    uint8_t battery_valid = 0u;
    uint8_t ibus_valid = 0u;
    const input_status_t *input = input_driver_get_status();

    if (out == NULL)
    {
        return;
    }

    /* This is the single aggregation point for real runtime state. Avoid mixing
     * in host intent here; telemetry must describe what hardware actually did. */
    memset(out, 0, sizeof(*out));

    drive_feedback_snapshot(&drive_feedback);
    battery_valid = (BatteryHandler_Snapshot(&battery) == 0u) ? 1u : 0u;
    ibus_valid = (ibus_handler_snapshot(&ibus) == 0u) ? 1u : 0u;

    out->left_wheel_angular_mrad_s = drive_feedback.left_wheel_angular_mrad_s;
    out->right_wheel_angular_mrad_s = drive_feedback.right_wheel_angular_mrad_s;
    out->z_lift_mm = telemetry_service_joint_position_mm((uint8_t)MOTOR_INDEX_JOINT);
    out->status_word = telemetry_service_build_status_word(&battery, battery_valid);
    out->health_word = telemetry_service_build_health_word(&battery,
                                                           battery_valid,
                                                           &ibus,
                                                           ibus_valid,
                                                           input);
    out->alarm_info = telemetry_service_build_alarm_info();
    out->batt_soc_x100 = (battery_valid != 0u) ? battery.remain_capacity : 0u;
    /* Keep bucket volume invalid until a real sensor path exists instead of
     * publishing a placeholder that upstream might treat as trustworthy. */
    out->bucket_volume_ml = 0u;
    out->bucket_volume_valid = 0u;
}
