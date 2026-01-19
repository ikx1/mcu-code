/**
 * @file motor_task.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "motor_task.h"
#include "bsp_motor_handler.h"
#include "bsp_gpio_driver.h"
#include "speed_filter.h"
#include "can_queue.h"

#include "system_cfg.h"
#include "control_task.h"
#include "uart_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>

/********************************** Defines **********************************/
#define READY_FLAG	0x55

/********************************** Variables ********************************/
static ema_filter_t rotation_filter;
static ema_filter_t linear_filter;

 car_info_t car_info;

/* ===================== Host power command arbitration =====================
 * status_word bits (current protocol in control_task.c):
 *  - Right motor: bits[1:0]  (00 none, 01 enable, 10 disable)
 *  - Left  motor: bits[4:3]  (00 none, 01 enable, 10 disable)
 * Reset bits remain handled in control_task.c
 *
 * NOTE: Single-writer principle: only Motor_Task is allowed to call
 * enable/disable, to avoid conflicts between 'host disable' and local logic.
 */
typedef enum { PWR_NONE = 0, PWR_ENABLE = 1, PWR_DISABLE = 2 } pwr_cmd_t;

static inline pwr_cmd_t host_pwr_cmd_from_status_word(uint16_t sw, int motor_index)
{
    uint8_t v = 0;
    if (motor_index == 0) {
        v = (uint8_t)((sw >> 0) & 0x03);   /* right motor */
    } else if (motor_index == 1) {
        v = (uint8_t)((sw >> 3) & 0x03);   /* left motor */
    } else {
        v = 0;
    }

    if (v == 1) return PWR_ENABLE;
    if (v == 2) return PWR_DISABLE;
    return PWR_NONE;
}

static inline bool host_force_disable(uint16_t sw, int motor_index)
{
    return (host_pwr_cmd_from_status_word(sw, motor_index) == PWR_DISABLE);
}

/* edge-triggered apply to avoid spamming 0x6040 every 50ms */
static uint8_t s_last_want_enable[MOTOR_NUM] = {0};

static inline void motor_apply_power_edge(motor_driver_t *drv, int idx, bool want_enable)
{
    uint8_t want = want_enable ? 1u : 0u;
    if (s_last_want_enable[idx] == want) return;

    if (want_enable) {
        if (drv->enable) drv->enable(drv);
    } else {
        if (drv->set_speed) drv->set_speed(drv, 0);
        if (drv->disable)   drv->disable(drv);
    }

    s_last_want_enable[idx] = want;
}


/********************************** Functions ********************************/
const car_info_t* get_car_info_p(void)
{
	return &car_info;
}

void Can_Analy_Task(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1));
		can_poll_parse_loop();
		// vTaskDelay(1);
	}
}

void can_send_task(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	while(1)
	{
		can_queue_service();
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
	}
}


void Motor_Task(void *pvParameters)
{
    uint32_t lastWakeTime = xTaskGetTickCount();
    motor_driver_t *motor_ptr = get_motor_drivers_p();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));

        const input_status_t* status = input_driver_get_status();
        ROBOT_INFO robot = {0};
        robot_info_snapshot(&robot);
        uint16_t sw = robot.status_word;

        if (motor_ptr == NULL || status == NULL)
        {
            continue;
        }

        for (int i = 0; i < MOTOR_NUM; ++i)
        {
            motor_driver_t *drv = &motor_ptr[i];


            if (drv->info == NULL) {
                continue;
            }
			/* Motor might power on after MCU init; re-run init sequence on POWER_ON */
			if ((drv->init_state.initialized == INITED) &&
				(drv->info->state == MOTOR_STATE_UNKNOWN)) 
			{
				drv->init_state.initialized = NOT_INITED;
				drv->init_state.init_type = MOTOR_INIT_NMT_PREOP;
				drv->init_state.ready_flag = 0;
			}
            /* ---------------- init stage ---------------- */
            if (drv->init_state.initialized != INITED)
            {
                if (drv->init) drv->init(drv);
                continue;
            }
            /* host / emergency arbitration */
            const ROBOT_MODE_T* rm = get_robot_mode_p();
            bool in_slave = (rm != NULL) && (rm->robot_mode_main == MODE_REMOTE) && (rm->robot_mode_sub == SUB_SLAVE);
            bool host_fresh = (!in_slave) || uart_link_is_alive();
            bool allow_enable = (!status->emergency_flag) && host_fresh && (!host_force_disable(sw, i));

            if (drv->init_state.ready_flag != READY_FLAG)
            {
                switch (drv->info->state)
                {
                    case MOTOR_STATE_POWER_ON:
                        if (drv->enter_ready) drv->enter_ready(drv);
                        break;

                    case MOTOR_STATE_READY:
                        if (drv->disable) drv->disable(drv);
                        break;

                    case MOTOR_STATE_DISABLED:
                        /* IMPORTANT:
                         * - if host forces DISABLE, do NOT auto-enable (avoid conflict)
                         * - consider init 'ready' when staying disabled, so upper logic can run
                         */
                        if (allow_enable) {
                            if (drv->enable) drv->enable(drv);
                        } else {
                            drv->init_state.ready_flag = READY_FLAG;
                            s_last_want_enable[i] = 0;
                        }
                        break;

                    case MOTOR_STATE_READING:
                        drv->init_state.ready_flag = READY_FLAG;
                        /* once driver enters READING, enforce power state immediately */
                        motor_apply_power_edge(drv, i, allow_enable);
                        break;

                    default:
                        break;
                }

                continue;
            }

            /* ---------------- runtime stage ----------------
             * Single-writer principle: only Motor_Task calls enable/disable.
             * Edge-triggered: only send 0x6040 when desired state changes.
             */
            motor_apply_power_edge(drv, i, allow_enable);
        }
    }
}


void Robot_Speed_Task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    /* 整车线速度 / 角速度 EMA */
    ema_filter_init(&linear_filter,   0.2f);
    ema_filter_init(&rotation_filter, 0.2f);

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
		motor_driver_t *motor_ptr = get_motor_drivers_p();
		if (motor_ptr == NULL || motor_ptr[0].info == NULL || motor_ptr[1].info == NULL)
		{
			continue;
		}

		motor_info_t *info1 = motor_ptr[0].info;
		motor_info_t *info2 = motor_ptr[1].info;

        float wr = -info1->wheel_speed;
        float wl = info2->wheel_speed;

        car_info.wheel_speed_right = wr;
        car_info.wheel_speed_left  = wl;

        /* 1) 整车线速度滤波 */
        float v_linear_raw = 0.5f * (wr + wl);
        float v_linear_f   = ema_filter_update(&linear_filter, v_linear_raw);
        car_info.global_speed = (int32_t)(v_linear_f * 1000.0f);

        /* 2) 整车角速度滤波 */
        float rotation_raw      = (wr - wl) / WHEEL_WIDTH;
        float rotation_filtered = ema_filter_update(&rotation_filter, rotation_raw);
        const float ROT_MOISE_EPS = 0.01f;
        if (fabsf(rotation_filtered) < ROT_MOISE_EPS) {
            rotation_filtered = 0.0f;
        }
        car_info.global_roation = (int32_t)(rotation_filtered * 1000.0f);
    }
}
