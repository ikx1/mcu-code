#ifndef CONTROL_TASK_USE_WDT_MONITOR
#define CONTROL_TASK_USE_WDT_MONITOR    (0u)
#endif

/********************************** Includes *********************************/
#include "control_task.h"
#include "robot_mode_service.h"
#include "robot_motion_cmd.h"
#include "host_control_cmd.h"
#include "motor_task.h"
#include "bsp_motor_handler.h"
#include "bsp_ibus_handler.h"
#include "bsp_gpio_driver.h"
#include "bsp_handler_display.h"
#include "uart_task.h"
#if (CONTROL_TASK_USE_WDT_MONITOR != 0u)
#include "wdt_task.h"
#endif

#include "system_cfg.h"

#include "FreeRTOS.h"
#include "task.h"


#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

/********************************** Defines **********************************/
/*----------------- Control task period -----------------*/
#define CONTROL_TASK_PERIOD_MS          (10u)   /* 10ms control cadence keeps homing/control responsive. */
#define CONTROL_TASK_MODE_PERIOD_MS     (50u)   /* 50ms mode cadence preserves the original IBUS behavior. */
#define TURN_M                          (1091)
#define DEADZONE                        (8)
#define INPUT_MAX                       (100)
#define MAX_INPUT_VALUE                 (100)
#define MIN_INPUT_VALUE                 (-100)
#define CONTROL_TASK_JOINT_MANUAL_DEADZONE      (3)

/********************************** Variables ********************************/
typedef enum
{
    CONTROL_OK = 0,
    CONTROL_ERR,
    CONTROL_ERR_PARMETER,
    CONTROL_NULL_POINTER,
    CONTROL_NOT_INITIALIZED,
    CONTROL_ERR_UNKNOWN_TYPE,
    CONTROL_NO_CONTROL_FUNCTION,
} control_err_t;

typedef struct
{
    float high_speed;
    float high_roat;
    float low_speed;
    float low_roat;
} robot_speed_par_t;

static const robot_speed_par_t robot_speed_par = {
	.high_speed = 0.8f,     
    .high_roat  = 2.0f,     
    .low_speed  = 0.4f,     
    .low_roat   = 0.8f      
};

#ifndef DEBUG
	static ROBOT_MODE_T robot_mode_current = {
#else
	ROBOT_MODE_T robot_mode_current = {
#endif
    .robot_mode_main = MODE_READY,
    .robot_mode_sub = SUB_STOP,
    .robot_speed_level = SPEED_LOW
};

#ifndef DEBUG
	static ROBOT_MODE_T robot_mode_last = {
#else
	ROBOT_MODE_T robot_mode_last = {
#endif
    .robot_mode_main = MODE_UNKNOWN,
    .robot_mode_sub = SUB_UNKNOWN,
};

#ifndef DEBUG
    static robot_motion_cmd_t s_robot_motion_cmd = {0};
    /* Require at least one post-entry host drive/lift command before SUB_SLAVE can act. */
    static uint32_t s_slave_drive_seq_epoch = 0u;
    static uint32_t s_slave_lift_seq_epoch = 0u;
#else
    robot_motion_cmd_t s_robot_motion_cmd = {0};
    uint32_t s_slave_drive_seq_epoch = 0u;
    uint32_t s_slave_lift_seq_epoch = 0u;
#endif /* DEBUG */

#if (CONTROL_TASK_USE_WDT_MONITOR != 0u) && !defined(DEBUG)
static uint32_t contronl_task_id;
#endif
	
/********************************** Functions ********************************/
static void robot_mode_update_50ms(void);
static uint8_t manu_set_lr_spd_low(void);
static uint8_t manu_set_lr_spd_high(void);
static void robot_state_execute(void);
static void control_task_apply_slave_motion(void);
static void control_task_apply_manual_joint_motion(void);
#ifdef DEBUG
static void control_task_apply_debug_host_joint_motion(void);
#endif
static bool control_task_host_slave_without_ibus_active(void);
static void control_task_joint_home_request_update_10ms(bool emergency);
static void control_task_set_mode_main_sub(ROBOT_MODE_MAIN main,
                                           ROBOT_MODE_SUB sub);
static void control_task_set_mode_main(ROBOT_MODE_MAIN main);
static void control_task_set_sub_and_speed(ROBOT_MODE_SUB sub,
                                           ROBOT_SPEED_LEVEL speed_level);
static void control_task_capture_slave_motion_epoch(void);
static void control_task_manual_joint_emit_reset(void);
static int32_t control_task_joint_mm_to_pos(int32_t lift_target_mm);


void ContronlTask(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t div_50ms = 0;

#if (CONTROL_TASK_USE_WDT_MONITOR != 0u) && !defined(DEBUG)
    (void)wdt_monitor_task_register("control", 300u, &contronl_task_id);
    (void)wdt_monitor_task_enable(contronl_task_id);
#endif

    display_handler_set_robot_mode((uint8_t)robot_mode_current.robot_mode_main,
                                   (uint8_t)robot_mode_current.robot_mode_sub);

    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));

#if (CONTROL_TASK_USE_WDT_MONITOR != 0u) && !defined(DEBUG)
        (void)wdt_monitor_task_feed(contronl_task_id);
#endif

        /* Keep one 10ms control loop and divide out the slower 50ms mode logic
         * instead of splitting mode/control ownership across different tasks. */
        const input_status_t* status = input_driver_get_status();
        bool emergency = (status != NULL) ? (status->emergency_flag != 0) : false;
        control_task_joint_home_request_update_10ms(emergency);
        display_handler_poll();

        /* Mode decisions stay at 50ms to preserve the original IBUS behavior. */
        if (++div_50ms >= (CONTROL_TASK_MODE_PERIOD_MS / CONTROL_TASK_PERIOD_MS))
        {
            div_50ms = 0;
            robot_mode_update_50ms();
        }

        /* Execute the active mode every 10ms so host/manual motion stays aligned with UART1 100Hz I/O. */
        robot_state_execute();
    }
}

void robot_motion_cmd_snapshot(robot_motion_cmd_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_robot_motion_cmd;
    taskEXIT_CRITICAL();
}

void robot_motion_cmd_clear_motion(void)
{
    taskENTER_CRITICAL();
    s_robot_motion_cmd.right_rpm = 0;
    s_robot_motion_cmd.left_rpm = 0;
    s_robot_motion_cmd.joint_cmd_type = ROBOT_JOINT_CMD_NONE;
    s_robot_motion_cmd.joint_cmd_value = 0;
    taskEXIT_CRITICAL();
}

void robot_motion_cmd_set_drive_rpm(int32_t right_rpm, int32_t left_rpm)
{
    taskENTER_CRITICAL();
    s_robot_motion_cmd.right_rpm = right_rpm;
    s_robot_motion_cmd.left_rpm = left_rpm;
    taskEXIT_CRITICAL();
}

void robot_motion_cmd_set_joint_absolute(int32_t joint_abs_pos)
{
    taskENTER_CRITICAL();
    s_robot_motion_cmd.joint_cmd_type = ROBOT_JOINT_CMD_ABSOLUTE_POS;
    s_robot_motion_cmd.joint_cmd_value = joint_abs_pos;
    taskEXIT_CRITICAL();
}

void robot_motion_cmd_set_joint_relative(int32_t joint_rel_pos)
{
    taskENTER_CRITICAL();
    s_robot_motion_cmd.joint_cmd_type = ROBOT_JOINT_CMD_RELATIVE_POS;
    s_robot_motion_cmd.joint_cmd_value = joint_rel_pos;
    taskEXIT_CRITICAL();
}

void robot_motion_cmd_clear_joint(void)
{
    taskENTER_CRITICAL();
    s_robot_motion_cmd.joint_cmd_type = ROBOT_JOINT_CMD_NONE;
    s_robot_motion_cmd.joint_cmd_value = 0;
    taskEXIT_CRITICAL();
}

void robot_mode_snapshot(ROBOT_MODE_T *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = robot_mode_current;
    taskEXIT_CRITICAL();
}

static void control_task_set_mode_main_sub(ROBOT_MODE_MAIN main,
                                           ROBOT_MODE_SUB sub)
{
    taskENTER_CRITICAL();
    robot_mode_current.robot_mode_main = main;
    robot_mode_current.robot_mode_sub = sub;
    taskEXIT_CRITICAL();

    display_handler_set_robot_mode((uint8_t)main, (uint8_t)sub);
}

static void control_task_set_mode_main(ROBOT_MODE_MAIN main)
{
    ROBOT_MODE_SUB sub;

    taskENTER_CRITICAL();
    robot_mode_current.robot_mode_main = main;
    sub = robot_mode_current.robot_mode_sub;
    taskEXIT_CRITICAL();

    display_handler_set_robot_mode((uint8_t)main, (uint8_t)sub);
}

static void control_task_set_sub_and_speed(ROBOT_MODE_SUB sub,
                                           ROBOT_SPEED_LEVEL speed_level)
{
    ROBOT_MODE_MAIN main;

    taskENTER_CRITICAL();
    robot_mode_current.robot_mode_sub = sub;
    robot_mode_current.robot_speed_level = speed_level;
    main = robot_mode_current.robot_mode_main;
    taskEXIT_CRITICAL();

    display_handler_set_robot_mode((uint8_t)main, (uint8_t)sub);
}


static inline bool joint_home_is_busy(void)
{
    return motor_task_joint_home_is_busy();
}

static inline void set_motor_speed(int32_t rs, int32_t ls)
{
    robot_motion_cmd_set_drive_rpm(rs, ls);
}

/* Host lift target is in mm; convert here to joint absolute position units. */
static inline void set_joint_location(int32_t lift_target_mm)
{
    int32_t pos = control_task_joint_mm_to_pos(lift_target_mm);
    robot_motion_cmd_set_joint_absolute(pos);
}

static inline void set_joint_pos(int32_t pos)
{
    (void)motor_cmd_set_relative_position(MOTOR_INDEX_JOINT, pos);
}

static void control_task_manual_joint_emit_reset(void)
{
    return;
}

static int32_t control_task_joint_mm_to_pos(int32_t lift_target_mm)
{
    return SYSTEM_CFG_JOINT_MM_TO_POS(lift_target_mm);
}

static void control_task_capture_slave_motion_epoch(void)
{
    host_control_cmd_t host_cmd = {0};

    host_control_cmd_snapshot(&host_cmd);
    /* Entering SUB_SLAVE snapshots the last seen host seq so stale commands that
     * were buffered before entry cannot move the robot. */
    s_slave_drive_seq_epoch = host_cmd.drive_cmd_seq;
    s_slave_lift_seq_epoch = host_cmd.lift_cmd_seq;
}

static void control_task_joint_home_request_update_10ms(bool emergency)
{
    motor_joint_home_status_t joint_home = {0};
    bool request_home = false;

    motor_task_joint_home_status_snapshot(&joint_home);

    /* Align with the demo slide-axis behavior: if the joint still needs homing,
     * start it automatically after power-up unless emergency stop is active. */
    request_home = (!emergency) &&
                   (joint_home.need_home != 0u) &&
                   (joint_home.fault == 0u);
    motor_task_joint_home_request_set(request_home);
}

static control_err_t motor_stop_all(void)
{
    robot_motion_cmd_clear_motion();

    return CONTROL_OK;
}

static control_err_t motorinfo_clear(void)
{
#ifdef DEBUG	
	if(MODE_READY == robot_mode_current.robot_mode_main)
	{
        robot_motion_cmd_clear_motion();

		return CONTROL_OK;
	}
#endif
	
    if(SUB_SLAVE != robot_mode_last.robot_mode_sub || 
      SUB_SLAVE != robot_mode_current.robot_mode_sub)
    {
        return CONTROL_OK;
    }

    robot_motion_cmd_clear_motion();

    return CONTROL_OK;
}

static control_err_t robot_set_mode(ROBOT_MODE_MAIN main, ROBOT_MODE_SUB sub)
{
    if (main >= MODE_UNKNOWN || sub >= SUB_UNKNOWN)
        return CONTROL_ERR;

    control_task_set_mode_main_sub(main, sub);
    return CONTROL_OK;
}

/* Main-mode transition handlers */
static control_err_t enter_emergency(void) 
{
    control_task_manual_joint_emit_reset();
    control_task_set_mode_main(MODE_EMERGENCY);

    return motor_stop_all();
}

static control_err_t enter_ready(void) 
{
    control_task_manual_joint_emit_reset();
    control_task_set_mode_main_sub(MODE_READY, SUB_STOP);

    return motor_stop_all();
}

static control_err_t enter_remote(void) 
{
    control_task_manual_joint_emit_reset();
    control_task_set_mode_main_sub(MODE_REMOTE, SUB_STOP);

    if(CONTROL_OK != motorinfo_clear())
    {
        return CONTROL_ERR;
    }

    return motor_stop_all();
}

static void update_remote_sub_mode(int16_t ltr5, int16_t ltr6, int16_t ltr7, int16_t ltr8) 
{
    ROBOT_MODE_SUB sub = SUB_STOP;
    ROBOT_SPEED_LEVEL speed_level = (ltr5 == 1) ? SPEED_HIGH : SPEED_LOW;

    /* Decode the four IBUS switches into one remote sub-mode in a single place
     * so every caller observes the same switch-to-behavior mapping. */
    if (ltr8 == 1 && ltr6 == 0 && ltr7 == 0) 
    {
        sub = SUB_CHARGE;
    } 
    else if (ltr6 == 0 && ltr7 == 0) 
    {
        sub = SUB_STOP;
    } 
    else if (ltr6 == 1 && ltr7 == 0) 
    {
        sub = SUB_MANUAL;
    }
    else if (ltr6 == 2 && ltr7 == 0) 
    {
        sub = SUB_SLAVE;
    } 
    else if (ltr6 == 0 && ltr7 == 1) 
    {
        sub = SUB_MOTOR_CTRL;
    } 
    else if (ltr6 == 0 && ltr7 == 2) 
    {
        sub = SUB_MOTOR_ERR_CLR;
    }

    control_task_set_sub_and_speed(sub, speed_level);
}

static control_err_t handle_remote_submode(void) 
{
	control_err_t ret = CONTROL_OK;

	switch (robot_mode_current.robot_mode_sub) 
	{
		case SUB_STOP:
            robot_motion_cmd_clear_motion();
			return CONTROL_OK;

		case SUB_MANUAL:
			ret = motorinfo_clear();
			if(CONTROL_OK != ret)
			{
				return ret;
			}	

			(robot_mode_current.robot_speed_level == SPEED_LOW) ? 
						manu_set_lr_spd_low() : manu_set_lr_spd_high();
			break;

		case SUB_SLAVE:
			control_task_apply_slave_motion();
			break;
		
		case SUB_CHARGE:
            robot_motion_cmd_clear_motion();
			break;

        case SUB_MOTOR_CTRL:
            if (!joint_home_is_busy()) {
                control_task_apply_manual_joint_motion();
            }
            break;

        case SUB_MOTOR_ERR_CLR:
            /* Error-clear mode owns the slide axis recovery path and should not
             * leave any stale drive/joint motion commands active. */
            robot_motion_cmd_clear_motion();
            break;

		default:
			break;
	}

	return ret;
}

static void robot_state_execute(void) 
{
    control_err_t err = CONTROL_OK;

    /* Main-mode entry handlers run only on transitions; steady-state motion stays
     * in the 10ms path below so manual and host commands share one cadence. */
    if (robot_mode_current.robot_mode_main != robot_mode_last.robot_mode_main)
    {
        switch (robot_mode_current.robot_mode_main)
        {
            case MODE_READY:     err = enter_ready(); break;
            case MODE_REMOTE:    err = enter_remote(); break;
            case MODE_EMERGENCY: err = enter_emergency(); break;
            default:             break;
        }
        if (err != CONTROL_OK)
		{
			// LOG_ERROR("Main mode switch failed: %d", err);
		}

        robot_mode_last.robot_mode_main = robot_mode_current.robot_mode_main;
        robot_mode_last.robot_mode_sub  = robot_mode_current.robot_mode_sub;
    }

	if (robot_mode_current.robot_mode_main == MODE_REMOTE)
	{
		if(robot_mode_current.robot_mode_sub != robot_mode_last.robot_mode_sub)
		{
            control_task_manual_joint_emit_reset();
            robot_motion_cmd_clear_joint();

            if (robot_mode_current.robot_mode_sub == SUB_SLAVE)
            {
                /* Require one fresh host command after taking slave ownership. */
                control_task_capture_slave_motion_epoch();
            }
			robot_mode_last.robot_mode_sub = robot_mode_current.robot_mode_sub;
		}

        err = handle_remote_submode();
        if (err != CONTROL_OK)
		{
            // LOG_ERROR("Sub mode handler failed: %d", err);
		}
	} 

#ifdef DEBUG
    if (robot_mode_current.robot_mode_main == MODE_READY)
    {
        control_task_apply_debug_host_joint_motion();
    }
#endif
}

static void robot_mode_update_50ms(void) 
{
    int16_t ltr5, ltr6, ltr7, ltr8;
    ROBOT_MODE_T mode = {
        .robot_mode_main = MODE_UNKNOWN,
        .robot_mode_sub = SUB_UNKNOWN,
        .robot_speed_level = SPEED_LOW,
    };

    IBUS_Handler ibus_data = {0};
	const input_status_t* status = input_driver_get_status();
	if((ibus_handler_snapshot(&ibus_data) != 0u) || (NULL == status))
	{
		return;
	}
    robot_mode_snapshot(&mode);

    /* Emergency has highest priority. When it is clear, IBUS connectivity decides
     * whether the robot sits in READY or accepts REMOTE mode selection. */
    if (status->emergency_flag) 
    {
        if(MODE_EMERGENCY != mode.robot_mode_main)
        {
            robot_set_mode(MODE_EMERGENCY, SUB_STOP);
        }
        return;
    }

    if(ibus_handler_get_channel_value(&ibus_data, IBUS_CH_CONN)) 
    {
        if (mode.robot_mode_main != MODE_REMOTE) 
        {
            robot_set_mode(MODE_REMOTE, SUB_STOP);
        }

        ltr5 = ibus_handler_get_channel_value(&ibus_data, IBUS_CH_SWD);
        ltr6 = ibus_handler_get_channel_value(&ibus_data, IBUS_CH_SWC);
        ltr7 = ibus_handler_get_channel_value(&ibus_data, IBUS_CH_SWB);
        ltr8 = ibus_handler_get_channel_value(&ibus_data, IBUS_CH_SWA);

        update_remote_sub_mode(ltr5, ltr6, ltr7, ltr8);
    } 
    else if (control_task_host_slave_without_ibus_active())
    {
        if ((mode.robot_mode_main != MODE_REMOTE) ||
            (mode.robot_mode_sub != SUB_SLAVE))
        {
            robot_set_mode(MODE_REMOTE, SUB_SLAVE);
        }
    }
    else 
    {
        if(MODE_READY != mode.robot_mode_main)
        {
            robot_set_mode(MODE_READY, SUB_STOP);
        }
    }

}

static bool control_task_host_slave_without_ibus_active(void)
{
#if (SYSTEM_CFG_ALLOW_HOST_SLAVE_WITHOUT_IBUS != 0u)
    /* No-IBUS host takeover is intentionally conservative: only an active host
     * command stream may claim SUB_SLAVE ownership. */
    return uart_session_is_alive() &&
           (uart_drive_cmd_is_fresh() || uart_lift_cmd_is_fresh());
#else
    return false;
#endif
}

static uint8_t manu_set_lr_spd_high(void)
{
	uint8_t rtn = 0;

    IBUS_Handler ibus_data = {0};
    if (ibus_handler_snapshot(&ibus_data) != 0u)
    {
        return 1u;
    }
	
	int16_t linear_input = ibus_handler_get_channel_value(&ibus_data, 
                                                                   IBUS_CH_RY);
    int16_t angular_input = ibus_handler_get_channel_value(&ibus_data, 
                                                                   IBUS_CH_RX);
    /* Deadzone handling */
	if (abs(linear_input) < DEADZONE) linear_input = 0;
    if (abs(angular_input) < DEADZONE) angular_input = 0;
	
	float linear_ratio = (float)linear_input / MAX_INPUT_VALUE;
    float angular_ratio = (float)angular_input / MAX_INPUT_VALUE;	
	
	float line_speed = robot_speed_par.high_speed * linear_ratio;
	float rotation_speed = robot_speed_par.high_roat * angular_ratio;
		
	float differential = rotation_speed * WHEEL_WIDTH / 2.0f;
    float left_speed = line_speed + differential;
    float right_speed = line_speed - differential;
	
	float max_rpm = robot_speed_par.high_speed * TURN_M;
    float left_rpm = left_speed * TURN_M;
    float right_rpm = right_speed * TURN_M;
    
    left_rpm = fmaxf(fminf(left_rpm, max_rpm), -max_rpm);
    right_rpm = fmaxf(fminf(right_rpm, max_rpm), -max_rpm);
	
	set_motor_speed(-(int32_t)right_rpm, (int32_t)left_rpm);

    return rtn;
}

static uint8_t manu_set_lr_spd_low(void)
{
	uint8_t rtn = 0;

    IBUS_Handler ibus_data = {0};
    if (ibus_handler_snapshot(&ibus_data) != 0u)
    {
        return 1u;
    }
	
	int16_t linear_input = ibus_handler_get_channel_value(&ibus_data, 
                                                                   IBUS_CH_RY);
    int16_t angular_input = ibus_handler_get_channel_value(&ibus_data, 
                                                                   IBUS_CH_RX);
    /* Deadzone handling */
	if (abs(linear_input) < DEADZONE) linear_input = 0;
    if (abs(angular_input) < DEADZONE) angular_input = 0;
	
	float linear_ratio = (float)linear_input / MAX_INPUT_VALUE;
    float angular_ratio = (float)angular_input / MAX_INPUT_VALUE;	
	
	float line_speed = robot_speed_par.low_speed * linear_ratio;
	float rotation_speed = robot_speed_par.low_roat * angular_ratio;
		
	float differential = rotation_speed * WHEEL_WIDTH / 2.0f;
    float left_speed = line_speed + differential;
    float right_speed = line_speed - differential;
	
	float max_rpm = robot_speed_par.low_speed * TURN_M;
    float left_rpm = left_speed * TURN_M;
    float right_rpm = right_speed * TURN_M;
    
    left_rpm = fmaxf(fminf(left_rpm, max_rpm), -max_rpm);
    right_rpm = fmaxf(fminf(right_rpm, max_rpm), -max_rpm);
	
	set_motor_speed(-(int32_t)right_rpm, (int32_t)left_rpm);

    return rtn;
}

static void control_task_apply_slave_motion(void)
{	
	host_control_cmd_t host_cmd = {0};
    bool drive_cmd_ready;
    bool lift_cmd_ready;
    float left_wheel_angular_rad_s;
    float right_wheel_angular_rad_s;

    host_control_cmd_snapshot(&host_cmd);
    /* SUB_SLAVE requires both freshness and a seq newer than the entry epoch, so
     * reusing the last mailbox value after mode switch or timeout cannot move. */
    drive_cmd_ready = uart_drive_cmd_is_fresh() &&
                      (host_cmd.drive_cmd_seq != s_slave_drive_seq_epoch);
    lift_cmd_ready = uart_lift_cmd_is_fresh() &&
                     (host_cmd.lift_cmd_seq != s_slave_lift_seq_epoch);

	int32_t lift_target_mm = host_cmd.z_lift_mm;

    left_wheel_angular_rad_s =
        ((float)host_cmd.left_wheel_angular_mrad_s) / 1000.0f;
	right_wheel_angular_rad_s =
        ((float)host_cmd.right_wheel_angular_mrad_s) / 1000.0f;
	
    if (!drive_cmd_ready)
    {
        set_motor_speed(0, 0);
    }
    else
    {
        /* Host sends mrad/s. Convert to rad/s, then wheel rpm, then scale by the
         * drivetrain reduction before clamping to the platform limit. */
        int32_t cmdL = ANGULAR_TO_RPM(left_wheel_angular_rad_s) * DRIVE_GEAR_RATIO;
        int32_t cmdR = ANGULAR_TO_RPM(right_wheel_angular_rad_s) * DRIVE_GEAR_RATIO;

        if(cmdL > TURN_M) cmdL = TURN_M;
        if(cmdR > TURN_M) cmdR  = TURN_M;
        if(cmdL < -TURN_M) cmdL = -TURN_M;
        if(cmdR < -TURN_M) cmdR  = -TURN_M;

        set_motor_speed(-(int32_t)cmdR, (int32_t)cmdL);
    }

    if (!lift_cmd_ready)
    {
        robot_motion_cmd_clear_joint();
        return;
    }

	if (motor_task_joint_is_homed() && !motor_task_joint_home_is_busy())
	{
        /* Absolute lift targets are only meaningful after homing establishes zero. */
		set_joint_location(lift_target_mm);
	}
    else
    {
        robot_motion_cmd_clear_joint();
    }
}

static void control_task_apply_manual_joint_motion(void)
{
    const input_status_t* status;
    IBUS_Handler ibus_data = {0};
    int16_t linear_input;
    int32_t step_pos;

    /* Manual joint control bypasses robot_motion_cmd and talks to the drive
     * directly, so clear any stale queued joint command first. */
    robot_motion_cmd_clear_joint();

    if (!motor_task_joint_is_homed())
    {
        set_joint_pos(0);
        return;
    }

    status = input_driver_get_status();

    if ((ibus_handler_snapshot(&ibus_data) != 0u) || (status == NULL))
    {
        return;
    }

    linear_input = ibus_handler_get_channel_value(&ibus_data, IBUS_CH_VRA);
    if ((linear_input > -CONTROL_TASK_JOINT_MANUAL_DEADZONE) &&
        (linear_input < CONTROL_TASK_JOINT_MANUAL_DEADZONE))
    {
        linear_input = 0;
    }

    step_pos = ((int32_t)linear_input * (int32_t)SYSTEM_CFG_JOINT_MANUAL_STEP_MAX) /
               INPUT_MAX;

    if (((step_pos > 0) && (status->limit_upper == 0u)) ||
        ((step_pos < 0) && (status->limit_lower == 0u)))
    {
        set_joint_pos(step_pos);
    }
    else
    {
        set_joint_pos(0);
    }
}

#ifdef DEBUG
static void control_task_apply_debug_host_joint_motion(void)
{
    host_control_cmd_t host_cmd = {0};
    int32_t lift_target_mm;

    if (!uart_lift_cmd_is_fresh())
    {
        robot_motion_cmd_clear_joint();
        return;
    }

    if (!motor_task_joint_is_homed() || motor_task_joint_home_is_busy())
    {
        robot_motion_cmd_clear_joint();
        return;
    }

    host_control_cmd_snapshot(&host_cmd);
    lift_target_mm = host_cmd.z_lift_mm;
    set_joint_location(lift_target_mm);
}
#endif


