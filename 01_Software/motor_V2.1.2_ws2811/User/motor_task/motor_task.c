/**
 * @file motor_task.c
 * @author 鏈啘 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "motor_task.h"
#include "drive_feedback_service.h"
#include "bsp_motor_handler.h"
#include "bsp_gpio_driver.h"

#include "system_cfg.h"
#include "robot_mode_service.h"
#include "robot_motion_cmd.h"
#include "host_control_cmd.h"
#include "uart_task.h"
#include "irq_guard.h"
#include "can_irq_adapter.h"
#include "can_queue.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <string.h>

/********************************** Defines **********************************/
#define READY_FLAG	0x55
#define MOTOR_TASK_PERIOD_MS            (10u)
#define DRIVE_FEEDBACK_PERIOD_MS        (10u)
#define DRIVE_ALARM_ERR_QUERY_RETRY_MS  (200u)
#define DRIVE_ERR_CLR_RESET_RETRY_MS    (200u)
#define JOINT_ERR_CLR_RESET_RETRY_MS    (200u)


/********************************** Variables ********************************/
// static ema_filter_t rotation_filter;
// static ema_filter_t linear_filter;

#ifndef DEBUG
	static drive_feedback_t s_drive_feedback;
#else
	drive_feedback_t s_drive_feedback;
#endif

/********************************** Functions ********************************/
static const motor_index_t s_drive_motor_indices[] = {
    MOTOR_INDEX_RIGHT,
    MOTOR_INDEX_LEFT,
};

typedef struct
{
    uint8_t request_enable;
    uint8_t need_home;
    uint8_t homed;
    uint8_t fault;
    motor_joint_home_state_t state;
    uint32_t start_ms;
    uint32_t settle_ms;
} motor_joint_home_ctrl_t;

static motor_joint_home_ctrl_t s_joint_home = {
#if (SYSTEM_CFG_JOINT_AUTO_HOME_ON_BOOT != 0u)
    .need_home = 1u,
#else
    .need_home = 0u,
    .homed = 1u,
    .state = MOTOR_JOINT_HOME_DONE,
#endif
#if (SYSTEM_CFG_JOINT_AUTO_HOME_ON_BOOT != 0u)
    .state = MOTOR_JOINT_HOME_IDLE,
#endif
};

static uint32_t drive_feedback_lock(void)
{
    return mcu_irq_guard_lock();
}

static void drive_feedback_unlock(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

static bool motor_task_joint_state_is_ready(motor_state_t state)
{
    return (state == JOINT_READY) || (state == MOTOR_STATE_POWER_ON);
}

static bool motor_task_joint_state_is_power_on(motor_state_t state)
{
    return (state == JOINT_POWER_ON) || (state == MOTOR_STATE_READY);
}

static bool motor_task_joint_state_is_enabled(motor_state_t state)
{
    return (state == JOINT_ENABLE) || ((uint16_t)state == 0x0433u);
}

static bool motor_task_joint_allow_motion(const input_status_t *status,
                                          const robot_motion_cmd_t *cmd,
                                          const host_control_cmd_t *host_cmd);
static bool motor_task_joint_allow_power_hold(const input_status_t *status,
                                              const host_control_cmd_t *host_cmd);

static uint32_t motor_task_joint_home_lock(void)
{
    return mcu_irq_guard_lock();
}

static void motor_task_joint_home_unlock(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

static bool motor_task_joint_home_state_busy(motor_joint_home_state_t state)
{
    return (state == MOTOR_JOINT_HOME_WAIT_MOTOR) ||
           (state == MOTOR_JOINT_HOME_SEEK_LOWER) ||
           (state == MOTOR_JOINT_HOME_SETTLE) ||
           (state == MOTOR_JOINT_HOME_ZERO);
}

static void motor_task_joint_home_store(const motor_joint_home_ctrl_t *ctrl)
{
    uint32_t primask;

    if (ctrl == NULL)
    {
        return;
    }

    primask = motor_task_joint_home_lock();
    s_joint_home.need_home = ctrl->need_home;
    s_joint_home.homed = ctrl->homed;
    s_joint_home.fault = ctrl->fault;
    s_joint_home.state = ctrl->state;
    s_joint_home.start_ms = ctrl->start_ms;
    s_joint_home.settle_ms = ctrl->settle_ms;
    motor_task_joint_home_unlock(primask);
}

void motor_task_joint_home_request_set(bool enable)
{
    uint32_t primask = motor_task_joint_home_lock();
    s_joint_home.request_enable = enable ? 1u : 0u;
    motor_task_joint_home_unlock(primask);
}

void motor_task_joint_home_status_snapshot(motor_joint_home_status_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = motor_task_joint_home_lock();
    out->request_enable = s_joint_home.request_enable;
    out->need_home = s_joint_home.need_home;
    out->homed = s_joint_home.homed;
    out->fault = s_joint_home.fault;
    out->state = s_joint_home.state;
    out->busy = motor_task_joint_home_state_busy(s_joint_home.state) ? 1u : 0u;
    motor_task_joint_home_unlock(primask);
}

bool motor_task_joint_is_homed(void)
{
    bool homed;
    uint32_t primask = motor_task_joint_home_lock();
    homed = (s_joint_home.homed != 0u);
    motor_task_joint_home_unlock(primask);
    return homed;
}

bool motor_task_joint_home_is_busy(void)
{
    bool busy;
    uint32_t primask = motor_task_joint_home_lock();
    busy = motor_task_joint_home_state_busy(s_joint_home.state);
    motor_task_joint_home_unlock(primask);
    return busy;
}

static inline host_motor_power_cmd_t host_motor_power_cmd_get(const host_control_cmd_t *host_cmd,
                                                              motor_index_t motor_index)
{
    if (host_cmd == NULL)
    {
        return HOST_MOTOR_POWER_CMD_NONE;
    }

    switch (motor_index)
    {
        case MOTOR_INDEX_RIGHT:
            return (host_motor_power_cmd_t)host_cmd->right_motor_power_cmd;

        case MOTOR_INDEX_LEFT:
            return (host_motor_power_cmd_t)host_cmd->left_motor_power_cmd;

        case MOTOR_INDEX_JOINT:
            return (host_motor_power_cmd_t)host_cmd->joint_motor_power_cmd;

        default:
            return HOST_MOTOR_POWER_CMD_NONE;
    }
}

static inline bool host_force_disable(const host_control_cmd_t *host_cmd,
                                      motor_index_t motor_index)
{
    return (host_motor_power_cmd_get(host_cmd, motor_index) ==
            HOST_MOTOR_POWER_CMD_DISABLE);
}

static bool motor_task_host_drive_policy_active(void)
{
    ROBOT_MODE_T rm;

    robot_mode_snapshot(&rm);
    /* Host power/reset policy is only authoritative in remote slave mode. */
    return (rm.robot_mode_main == MODE_REMOTE) &&
           (rm.robot_mode_sub == SUB_SLAVE);
}

/* Track requested power/reset edges so the 10ms task only emits control words
 * when policy actually changes. */
static uint8_t s_last_want_enable[MOTOR_NUM] = {0};
static uint8_t s_last_reset_mask = 0u;
static uint8_t s_drive_alarm_query_active[MOTOR_NUM] = {0};
static uint32_t s_drive_alarm_query_last_ms[MOTOR_NUM] = {0};
static uint8_t s_drive_err_clear_host_active[MOTOR_NUM] = {0};
static uint32_t s_drive_err_clear_last_reset_ms[MOTOR_NUM] = {0};
static uint8_t s_joint_err_clear_active_last = 0u;
static uint8_t s_joint_err_clear_alarm_seen = 0u;
static uint32_t s_joint_err_clear_last_reset_ms = 0u;
static inline void motor_apply_power_edge(motor_index_t idx,
                                          bool want_enable)
{
    uint8_t want = want_enable ? 1u : 0u;
    if (s_last_want_enable[(uint32_t)idx] == want) return;

    if (want_enable) {
        (void)motor_cmd_enable(idx);
    } else {
        (void)motor_cmd_set_speed(idx, 0);
        (void)motor_cmd_disable(idx);
    }

    s_last_want_enable[(uint32_t)idx] = want;
}

static void motor_task_drive_alarm_query(motor_index_t idx,
                                         const motor_runtime_t *rt)
{
    uint32_t idx_u32 = (uint32_t)idx;
    uint32_t now_ms;

    if ((rt == NULL) || (idx_u32 >= (uint32_t)MOTOR_NUM))
    {
        return;
    }

    if ((idx != MOTOR_INDEX_RIGHT) && (idx != MOTOR_INDEX_LEFT))
    {
        return;
    }

    if (rt->state != MOTOR_STATE_ALARM)
    {
        s_drive_alarm_query_active[idx_u32] = 0u;
        s_drive_alarm_query_last_ms[idx_u32] = 0u;
        return;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if ((s_drive_alarm_query_active[idx_u32] == 0u) ||
        ((now_ms - s_drive_alarm_query_last_ms[idx_u32]) >= DRIVE_ALARM_ERR_QUERY_RETRY_MS))
    {
        (void)motor_cmd_inquire_state_err(idx);
        s_drive_alarm_query_active[idx_u32] = 1u;
        s_drive_alarm_query_last_ms[idx_u32] = now_ms;
    }
}

static void motor_task_apply_reset_edges(const host_control_cmd_t *host_cmd)
{
    uint8_t reset_mask = 0u;
    uint8_t rising_mask;
    uint32_t now_ms;

    if (host_cmd != NULL)
    {
        reset_mask = host_cmd->motor_reset_mask;
    }

    /* Reset is edge-triggered and only honored while the host session is alive,
     * otherwise a stale latched bit could reset the motor much later. */
    if (!motor_task_host_drive_policy_active() || !uart_session_is_alive())
    {
        s_last_reset_mask = 0u;
        s_drive_err_clear_host_active[(uint32_t)MOTOR_INDEX_RIGHT] = 0u;
        s_drive_err_clear_host_active[(uint32_t)MOTOR_INDEX_LEFT] = 0u;
        s_drive_err_clear_last_reset_ms[(uint32_t)MOTOR_INDEX_RIGHT] = 0u;
        s_drive_err_clear_last_reset_ms[(uint32_t)MOTOR_INDEX_LEFT] = 0u;
        return;
    }

    rising_mask = (uint8_t)(reset_mask & (uint8_t)(~s_last_reset_mask));
    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if ((rising_mask & ROBOT_RESET_RIGHT_BIT) != 0u)
    {
        s_drive_err_clear_host_active[(uint32_t)MOTOR_INDEX_RIGHT] = 1u;
        s_drive_err_clear_last_reset_ms[(uint32_t)MOTOR_INDEX_RIGHT] = now_ms;
        (void)motor_cmd_reset_motor(MOTOR_INDEX_RIGHT);
    }
    if ((rising_mask & ROBOT_RESET_LEFT_BIT) != 0u)
    {
        s_drive_err_clear_host_active[(uint32_t)MOTOR_INDEX_LEFT] = 1u;
        s_drive_err_clear_last_reset_ms[(uint32_t)MOTOR_INDEX_LEFT] = now_ms;
        (void)motor_cmd_reset_motor(MOTOR_INDEX_LEFT);
    }
    if ((rising_mask & ROBOT_RESET_JOINT_BIT) != 0u)
    {
        (void)motor_cmd_reset_motor(MOTOR_INDEX_JOINT);
    }

    s_last_reset_mask = reset_mask;
}

static void motor_task_apply_drive_motion(const robot_motion_cmd_t *cmd,
                                          const input_status_t *status,
                                          const host_control_cmd_t *host_cmd)
{
    int32_t right_rpm = 0;
    int32_t left_rpm = 0;
    bool host_policy_active = motor_task_host_drive_policy_active();

    if (cmd != NULL)
    {
        right_rpm = cmd->right_rpm;
        left_rpm = cmd->left_rpm;
    }

    /* This is the final execution-side safety gate: emergency or host-forced
     * disable always wins, regardless of what the control task requested. */
    if ((status == NULL) || (status->emergency_flag != 0u))
    {
        right_rpm = 0;
        left_rpm = 0;
    }

    if (host_policy_active &&
        host_force_disable(host_cmd, MOTOR_INDEX_RIGHT))
    {
        right_rpm = 0;
    }
    if (host_policy_active &&
        host_force_disable(host_cmd, MOTOR_INDEX_LEFT))
    {
        left_rpm = 0;
    }

    (void)motor_cmd_set_speed(MOTOR_INDEX_RIGHT, right_rpm);
    (void)motor_cmd_set_speed(MOTOR_INDEX_LEFT, left_rpm);
}

static bool motor_task_joint_err_clear_active(void)
{
    ROBOT_MODE_T rm;

    robot_mode_snapshot(&rm);
    return (rm.robot_mode_main == MODE_REMOTE) &&
           (rm.robot_mode_sub == SUB_MOTOR_ERR_CLR);
}

static bool motor_task_process_drive_err_clear(motor_index_t idx,
                                               const input_status_t *status,
                                               const motor_runtime_t *rt)
{
    uint32_t idx_u32 = (uint32_t)idx;
    bool active;
    uint32_t now_ms;

    if ((idx != MOTOR_INDEX_RIGHT) && (idx != MOTOR_INDEX_LEFT))
    {
        return false;
    }

    active = motor_task_joint_err_clear_active() ||
             (s_drive_err_clear_host_active[idx_u32] != 0u);
    if (!active)
    {
        s_drive_err_clear_host_active[idx_u32] = 0u;
        s_drive_err_clear_last_reset_ms[idx_u32] = 0u;
        return false;
    }

    if (rt == NULL)
    {
        return true;
    }

    if ((status == NULL) || (status->emergency_flag != 0u))
    {
        s_last_want_enable[idx_u32] = 0u;
        return true;
    }

    if (rt->state == MOTOR_STATE_ALARM)
    {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if ((s_drive_err_clear_last_reset_ms[idx_u32] == 0u) ||
            ((now_ms - s_drive_err_clear_last_reset_ms[idx_u32]) >= DRIVE_ERR_CLR_RESET_RETRY_MS))
        {
            (void)motor_cmd_reset_motor(idx);
            s_drive_err_clear_last_reset_ms[idx_u32] = now_ms;
        }

        s_last_want_enable[idx_u32] = 0u;
        return true;
    }

    s_drive_err_clear_host_active[idx_u32] = 0u;
    s_drive_err_clear_last_reset_ms[idx_u32] = 0u;
    return false;
}

static void motor_task_joint_home_clear_fault(bool request_home_again)
{
    uint32_t primask = motor_task_joint_home_lock();

    if (s_joint_home.state == MOTOR_JOINT_HOME_FAULT)
    {
        s_joint_home.need_home = request_home_again ? 1u : s_joint_home.need_home;
        s_joint_home.homed = 0u;
        s_joint_home.fault = 0u;
        s_joint_home.state = MOTOR_JOINT_HOME_IDLE;
        s_joint_home.start_ms = 0u;
        s_joint_home.settle_ms = 0u;
    }

    motor_task_joint_home_unlock(primask);
}

static bool motor_task_process_joint_err_clear(const input_status_t *status,
                                               const motor_runtime_t *rt)
{
    bool active;
    uint32_t now_ms;

    active = motor_task_joint_err_clear_active();
    if (!active)
    {
        s_joint_err_clear_active_last = 0u;
        s_joint_err_clear_alarm_seen = 0u;
        s_joint_err_clear_last_reset_ms = 0u;
        return false;
    }

    if (rt == NULL)
    {
        return true;
    }

    if (s_joint_err_clear_active_last == 0u)
    {
        s_joint_err_clear_alarm_seen = 0u;
        s_joint_err_clear_last_reset_ms = 0u;
    }
    s_joint_err_clear_active_last = 1u;

    if ((status == NULL) || (status->emergency_flag != 0u))
    {
        s_last_want_enable[(uint32_t)MOTOR_INDEX_JOINT] = 0u;
        return true;
    }

    if (rt->state == JOINT_ALARM)
    {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        s_joint_err_clear_alarm_seen = 1u;

        if ((s_joint_err_clear_last_reset_ms == 0u) ||
            ((now_ms - s_joint_err_clear_last_reset_ms) >= JOINT_ERR_CLR_RESET_RETRY_MS))
        {
            (void)motor_cmd_reset_motor(MOTOR_INDEX_JOINT);
            s_joint_err_clear_last_reset_ms = now_ms;
        }

        s_last_want_enable[(uint32_t)MOTOR_INDEX_JOINT] = 0u;
        return true;
    }

    if (s_joint_err_clear_alarm_seen != 0u)
    {
        /* If homing faulted because the drive alarmed, clear the homing fault
         * latch here so the axis can be re-enabled and homed again later. */
        motor_task_joint_home_clear_fault(true);
        s_joint_err_clear_alarm_seen = 0u;
    }

    if (motor_task_joint_state_is_ready(rt->state))
    {
        (void)motor_cmd_disable(MOTOR_INDEX_JOINT);
        s_last_want_enable[(uint32_t)MOTOR_INDEX_JOINT] = 0u;
        return true;
    }

    if (motor_task_joint_state_is_power_on(rt->state) ||
        motor_task_joint_state_is_enabled(rt->state))
    {
        motor_apply_power_edge(MOTOR_INDEX_JOINT, true);
        return true;
    }

    return true;
}

static void motor_task_apply_joint_motion(const input_status_t *status,
                                          const robot_motion_cmd_t *cmd,
                                          const host_control_cmd_t *host_cmd)
{
    motor_runtime_t rt;

    if (cmd == NULL)
    {
        return;
    }

    if (!motor_task_joint_allow_motion(status, cmd, host_cmd))
    {
        return;
    }

    memset(&rt, 0, sizeof(rt));
    if (motor_runtime_snapshot(MOTOR_INDEX_JOINT, &rt) != 0u)
    {
        return;
    }

    if (rt.initialized != INITED)
    {
        return;
    }

    if (!motor_task_joint_state_is_enabled(rt.state))
    {
        return;
    }

    switch (cmd->joint_cmd_type)
    {
        case ROBOT_JOINT_CMD_ABSOLUTE_POS:
            (void)motor_cmd_set_absolute_position(MOTOR_INDEX_JOINT,
                                                  cmd->joint_cmd_value);
            robot_motion_cmd_clear_joint();
            break;

        case ROBOT_JOINT_CMD_RELATIVE_POS:
            (void)motor_cmd_set_relative_position(MOTOR_INDEX_JOINT,
                                                  cmd->joint_cmd_value);
            robot_motion_cmd_clear_joint();
            break;

        case ROBOT_JOINT_CMD_NONE:
        default:
            break;
    }
}

static bool motor_task_joint_allow_motion(const input_status_t *status,
                                          const robot_motion_cmd_t *cmd,
                                          const host_control_cmd_t *host_cmd)
{
    ROBOT_MODE_T rm;
    bool host_policy_active;
    bool debug_ready_host_lift_active = false;

    /* Motion execution stays strict even though power-hold is now relaxed:
     * there must be a real command, the axis must already be homed, and only
     * remote-owned motion or the DEBUG ready-mode host-lift path may run. */
    if ((status == NULL) || (cmd == NULL))
    {
        return false;
    }

    if (status->emergency_flag != 0u)
    {
        return false;
    }

    if (cmd->joint_cmd_type == ROBOT_JOINT_CMD_NONE)
    {
        return false;
    }

    if (!motor_task_joint_is_homed())
    {
        return false;
    }

    robot_mode_snapshot(&rm);
#ifdef DEBUG
    if ((rm.robot_mode_main == MODE_READY) &&
        (cmd->joint_cmd_type == ROBOT_JOINT_CMD_ABSOLUTE_POS) &&
        uart_lift_cmd_is_fresh())
    {
        debug_ready_host_lift_active = true;
    }
#endif

    if ((rm.robot_mode_main != MODE_REMOTE) && (!debug_ready_host_lift_active))
    {
        return false;
    }

    if (debug_ready_host_lift_active)
    {
        return !host_force_disable(host_cmd, MOTOR_INDEX_JOINT);
    }

    host_policy_active = (rm.robot_mode_sub == SUB_SLAVE);
    if (host_policy_active)
    {
        if (!uart_session_is_alive())
        {
            return false;
        }
        if (host_force_disable(host_cmd, MOTOR_INDEX_JOINT))
        {
            return false;
        }
    }

    return host_policy_active ||
           (rm.robot_mode_sub == SUB_MOTOR_CTRL);
}

static bool motor_task_joint_allow_power_hold(const input_status_t *status,
                                              const host_control_cmd_t *host_cmd)
{
    if (status == NULL)
    {
        return false;
    }

    if (status->emergency_flag != 0u)
    {
        return false;
    }

    /* Keep the slide axis enabled after the ready sequence so host/manual
     * position commands do not need to wait for a later mode-triggered enable.
     * A fresh host-side force-disable command still wins immediately. */
    if (uart_session_is_alive() &&
        host_force_disable(host_cmd, MOTOR_INDEX_JOINT))
    {
        return false;
    }

    return true;
}

static bool motor_task_drive_allow_enable(const input_status_t *status,
                                          const host_control_cmd_t *host_cmd,
                                          motor_index_t idx)
{
    ROBOT_MODE_T rm;
    bool mode_allows_power;
    bool host_policy_active;
    bool host_force_disable_active;
    robot_mode_snapshot(&rm);

    if (status == NULL)
    {
        return false;
    }

    /* READY can keep drive power only when the build explicitly opts in.
     * Host navigation (SUB_SLAVE) may also keep drive power after entry even if
     * IBUS or the host session later drops, but a fresh explicit disable still
     * wins immediately. */
    mode_allows_power = (rm.robot_mode_main == MODE_REMOTE);
#if (SYSTEM_CFG_READY_KEEP_DRIVE_POWER != 0u)
    mode_allows_power = mode_allows_power ||
                        (rm.robot_mode_main == MODE_READY);
#endif
    host_policy_active = (rm.robot_mode_main == MODE_REMOTE) &&
                         (rm.robot_mode_sub == SUB_SLAVE);
    host_force_disable_active = uart_session_is_alive() &&
                                host_force_disable(host_cmd, idx);

    return mode_allows_power &&
           (!status->emergency_flag) &&
           ((!host_policy_active) ||
#if (SYSTEM_CFG_HOST_SLAVE_KEEP_DRIVE_POWER != 0u)
            true
#else
            uart_session_is_alive()
#endif
            ) &&
           ((!host_policy_active) || (!host_force_disable_active));
}

static bool motor_task_drive_run_ready_sequence(const motor_runtime_t *rt,
                                                motor_index_t idx,
                                                bool allow_enable)
{
    bool ready = false;

    if (rt == NULL)
    {
        return false;
    }

    if (rt->ready_flag == READY_FLAG)
    {
        return true;
    }

    /* Drive motors must walk through the vendor-ready handshake once after init
     * before normal enable/disable edge control becomes reliable. */
    switch (rt->state)
    {
        case MOTOR_STATE_POWER_ON:
            (void)motor_cmd_enter_ready(idx);
            break;

        case MOTOR_STATE_READY:
            (void)motor_cmd_disable(idx);
            break;

        case MOTOR_STATE_DISABLED:
            if (allow_enable)
            {
                (void)motor_cmd_enable(idx);
            }
            else
            {
                if (motor_runtime_set_ready_flag(idx, READY_FLAG) == 0u)
                {
                    ready = true;
                }
                s_last_want_enable[(uint32_t)idx] = 0u;
            }
            break;

        case MOTOR_STATE_READING:
            if (motor_runtime_set_ready_flag(idx, READY_FLAG) == 0u)
            {
                ready = true;
                motor_apply_power_edge(idx, allow_enable);
            }
            break;

        default:
            break;
    }

    return ready;
}

static void motor_task_handle_drive_motor(motor_index_t idx,
                                          const input_status_t *status,
                                          const host_control_cmd_t *host_cmd)
{
    motor_runtime_t rt;
    memset(&rt, 0, sizeof(rt));

    if (motor_runtime_snapshot(idx, &rt) != 0u)
    {
        return;
    }

    if ((rt.initialized == INITED) &&
        (rt.state == MOTOR_STATE_UNKNOWN))
    {
        /* A runtime that falls back to UNKNOWN is treated as lost ownership and
         * forced through init again instead of trusting stale state. */
        if (motor_runtime_reset_init_state(idx) != 0u)
        {
            return;
        }
        rt.initialized = NOT_INITED;
    }

    if (rt.initialized != INITED)
    {
        (void)motor_cmd_init(idx);
        return;
    }

    motor_task_drive_alarm_query(idx, &rt);
    if (motor_task_process_drive_err_clear(idx, status, &rt))
    {
        return;
    }

    if (rt.state == MOTOR_STATE_ALARM)
    {
        s_last_want_enable[(uint32_t)idx] = 0u;
        return;
    }

    bool allow_enable = motor_task_drive_allow_enable(status, host_cmd, idx);
    if (!motor_task_drive_run_ready_sequence(&rt, idx, allow_enable))
    {
        return;
    }

    motor_apply_power_edge(idx, allow_enable);
}

static void motor_task_joint_stop_and_disable(void)
{
    (void)motor_cmd_set_speed(MOTOR_INDEX_JOINT, 0);
    (void)motor_cmd_disable(MOTOR_INDEX_JOINT);
}

static bool motor_task_process_joint_home(const input_status_t *status)
{
    motor_joint_home_ctrl_t ctrl;
    motor_runtime_t rt;
    uint32_t now_ms;
    bool should_run;
    bool override_joint_owner;
    uint32_t primask;

    primask = motor_task_joint_home_lock();
    ctrl = s_joint_home;
    motor_task_joint_home_unlock(primask);

    /* Homing temporarily overrides normal joint ownership. While it is pending
     * or faulted, no other joint path is allowed to issue position commands. */
    override_joint_owner = (ctrl.need_home != 0u) || (ctrl.state == MOTOR_JOINT_HOME_FAULT);
    if (!override_joint_owner)
    {
        return false;
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    should_run = (status != NULL) &&
                 (status->emergency_flag == 0u) &&
                 (ctrl.request_enable != 0u) &&
                 (ctrl.need_home != 0u);

    if (!should_run)
    {
        if (motor_task_joint_home_state_busy(ctrl.state))
        {
            /* Dropping the request or hitting emergency aborts the sequence and
             * leaves the joint in a known disabled state. */
            motor_task_joint_stop_and_disable();
            ctrl.homed = 0u;
            ctrl.fault = 0u;
            ctrl.state = MOTOR_JOINT_HOME_IDLE;
            ctrl.start_ms = 0u;
            ctrl.settle_ms = 0u;
        }

        motor_task_joint_home_store(&ctrl);
        return true;
    }

    memset(&rt, 0, sizeof(rt));

    switch (ctrl.state)
    {
        case MOTOR_JOINT_HOME_IDLE:
            /* Start a fresh homing transaction. */
            ctrl.homed = 0u;
            ctrl.fault = 0u;
            ctrl.start_ms = now_ms;
            ctrl.settle_ms = 0u;
            ctrl.state = MOTOR_JOINT_HOME_WAIT_MOTOR;
            break;

        case MOTOR_JOINT_HOME_WAIT_MOTOR:
            /* The slide axis follows a two-step state change from the demo:
             * JOINT_READY -> send 0x06, JOINT_POWER_ON -> send 0x07, then it can seek. */
            if (motor_runtime_snapshot(MOTOR_INDEX_JOINT, &rt) != 0u)
            {
                break;
            }

            if (rt.state == MOTOR_STATE_UNKNOWN)
            {
                (void)motor_cmd_init(MOTOR_INDEX_JOINT);
                break;
            }

            if (motor_task_joint_state_is_ready(rt.state))
            {
                (void)motor_cmd_disable(MOTOR_INDEX_JOINT);
                break;
            }

            if (motor_task_joint_state_is_power_on(rt.state))
            {
                (void)motor_cmd_enable(MOTOR_INDEX_JOINT);
                break;
            }

            if (motor_task_joint_state_is_enabled(rt.state))
            {
                ctrl.start_ms = now_ms;
                ctrl.state = MOTOR_JOINT_HOME_SEEK_LOWER;
                break;
            }

            if (rt.state == JOINT_ALARM)
            {
                motor_task_joint_stop_and_disable();
                ctrl.need_home = 0u;
                ctrl.homed = 0u;
                ctrl.fault = 1u;
                ctrl.state = MOTOR_JOINT_HOME_FAULT;
                break;
            }
            break;

        case MOTOR_JOINT_HOME_SEEK_LOWER:
            /* Drive toward the lower limit until the switch hits or timeout/fault
             * proves the zero search is unsafe. */
            if ((now_ms - ctrl.start_ms) > SYSTEM_CFG_JOINT_HOME_TIMEOUT_MS)
            {
                motor_task_joint_stop_and_disable();
                ctrl.need_home = 0u;
                ctrl.homed = 0u;
                ctrl.fault = 1u;
                ctrl.state = MOTOR_JOINT_HOME_FAULT;
                break;
            }

            if ((status != NULL) && (status->limit_lower != 0u))
            {
                motor_task_joint_stop_and_disable();
                ctrl.settle_ms = now_ms;
                ctrl.state = MOTOR_JOINT_HOME_SETTLE;
                break;
            }

            if (motor_cmd_set_relative_position(MOTOR_INDEX_JOINT,
                                                SYSTEM_CFG_JOINT_HOME_NEG_TARGET_POS_CMD) != 0u)
            {
                motor_task_joint_stop_and_disable();
                ctrl.need_home = 0u;
                ctrl.homed = 0u;
                ctrl.fault = 1u;
                ctrl.state = MOTOR_JOINT_HOME_FAULT;
            }
            break;

        case MOTOR_JOINT_HOME_SETTLE:
            /* Give the mechanism time to settle after touching the hard stop so
             * zeroing is not performed while the axis is still bouncing. */
            if ((now_ms - ctrl.settle_ms) >= SYSTEM_CFG_JOINT_HOME_SETTLE_MS)
            {
                ctrl.state = MOTOR_JOINT_HOME_ZERO;
            }
            break;

        case MOTOR_JOINT_HOME_ZERO:
            /* Reset encoder zero at the mechanical lower stop, then command a
             * zero relative move so higher layers can treat the axis as homed. */
            (void)motor_cmd_disable(MOTOR_INDEX_JOINT);
            if (motor_cmd_reset_zero(MOTOR_INDEX_JOINT) != 0u)
            {
                motor_task_joint_stop_and_disable();
                ctrl.need_home = 0u;
                ctrl.homed = 0u;
                ctrl.fault = 1u;
                ctrl.state = MOTOR_JOINT_HOME_FAULT;
                break;
            }

            (void)motor_cmd_enable(MOTOR_INDEX_JOINT);
            (void)motor_cmd_set_relative_position(MOTOR_INDEX_JOINT, 0);
            ctrl.need_home = 0u;
            ctrl.homed = 1u;
            ctrl.fault = 0u;
            ctrl.state = MOTOR_JOINT_HOME_DONE;
            break;

        case MOTOR_JOINT_HOME_DONE:
            /* Stay latched as homed until a new request or fault clears it. */
            ctrl.homed = 1u;
            break;

        case MOTOR_JOINT_HOME_FAULT:
        default:
            /* Fault keeps ownership here so a human can inspect/reset before any
             * automatic joint motion resumes. */
            ctrl.homed = 0u;
            break;
    }

    motor_task_joint_home_store(&ctrl);
    return true;
}

static void motor_task_handle_joint_motor(const input_status_t *status,
                                          const robot_motion_cmd_t *cmd,
                                          const host_control_cmd_t *host_cmd)
{
    motor_runtime_t rt;
    bool allow_enable;
    memset(&rt, 0, sizeof(rt));

    if (motor_runtime_snapshot(MOTOR_INDEX_JOINT, &rt) != 0u)
    {
        return;
    }

    /* Keep nudging the slide node through init until its first TPDO updates the
     * runtime state, then rely on the periodic auto-upload path from the drive. */
    if ((rt.initialized != INITED) || (rt.state == MOTOR_STATE_UNKNOWN))
    {
        (void)motor_cmd_init(MOTOR_INDEX_JOINT);
        return;
    }

    if (motor_task_process_joint_err_clear(status, &rt))
    {
        return;
    }

    /* Homing has higher priority than ordinary joint control because it defines
     * the only valid zero reference for later absolute-position commands. */
    if (motor_task_process_joint_home(status))
    {
        return;
    }

    allow_enable = motor_task_joint_allow_power_hold(status, host_cmd);

    if (rt.state == JOINT_ALARM)
    {
        s_last_want_enable[(uint32_t)MOTOR_INDEX_JOINT] = 0u;
        return;
    }

    switch (rt.state)
    {
        default:
            if (motor_task_joint_state_is_ready(rt.state))
            {
                (void)motor_cmd_disable(MOTOR_INDEX_JOINT);
                s_last_want_enable[(uint32_t)MOTOR_INDEX_JOINT] = 0u;
            }
            else if (motor_task_joint_state_is_power_on(rt.state) ||
                     motor_task_joint_state_is_enabled(rt.state))
            {
                motor_apply_power_edge(MOTOR_INDEX_JOINT, allow_enable);
            }
            break;
    }
}

void drive_feedback_snapshot(drive_feedback_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = drive_feedback_lock();
    *out = s_drive_feedback;
    drive_feedback_unlock(primask);
}

void Can_Analy_Task(void *pvParameters)
{
    (void)pvParameters;

	while(1)
	{
        /* This task is the convergence point for CAN recovery, frame parsing,
         * queued TX and UART/CAN diagnostic maintenance. */
        mcu_can_diag_poll_recover();
        uart_task_diag_maintenance();
		can_poll_parse_loop();
        (void)can_queue_kick_tx();
		vTaskDelay(1);
	}
}

void Motor_Task(void *pvParameters)
{
    (void)pvParameters;

	TickType_t lastWakeTime = xTaskGetTickCount();

    while(1)
    {	
        robot_motion_cmd_t motion_cmd = {0};

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
				
        /* Execution layer snapshots all upstream intent once per tick, then owns
         * the final ordering: reset edges, power state, drive motion, joint motion. */
        const input_status_t* status = input_driver_get_status();
        host_control_cmd_t host_cmd = {0};
        host_control_cmd_snapshot(&host_cmd);
        robot_motion_cmd_snapshot(&motion_cmd);
        if (status == NULL)
        {
            continue;
        }

        motor_task_apply_reset_edges(&host_cmd);

        for (uint32_t i = 0; i < (sizeof(s_drive_motor_indices) / sizeof(s_drive_motor_indices[0])); ++i)
        {
            motor_task_handle_drive_motor(s_drive_motor_indices[i],
                                          status,
                                          &host_cmd);
        }

        motor_task_handle_joint_motor(status, &motion_cmd, &host_cmd);
        motor_task_apply_drive_motion(&motion_cmd, status, &host_cmd);
        motor_task_apply_joint_motion(status, &motion_cmd, &host_cmd);
	}
}

void Robot_Speed_Task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();

    // ema_filter_init(&linear_filter,   0.2f);
    // ema_filter_init(&rotation_filter, 0.2f);

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DRIVE_FEEDBACK_PERIOD_MS));
		
		motor_info_t right_info = {0};
		motor_info_t left_info = {0};
		if ((motor_info_snapshot(MOTOR_INDEX_RIGHT, &right_info) != 0u) ||
            (motor_info_snapshot(MOTOR_INDEX_LEFT, &left_info) != 0u))
        {
            continue;
        }

        /* Publish measured wheel speed only. Telemetry should reflect what the
         * drive is actually doing, not the last commanded speed. */
        float wr1 = -right_info.wheel_angular;
        float wl1 = left_info.wheel_angular;
        uint32_t primask = drive_feedback_lock();
        s_drive_feedback.left_wheel_angular_mrad_s = (int32_t)(wl1 * 1000.0f);
        s_drive_feedback.right_wheel_angular_mrad_s = (int32_t)(wr1 * 1000.0f);
        drive_feedback_unlock(primask);
    }
}

