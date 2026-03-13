/**
 * @file ws2812_task.c
 * @author 
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "ws2812_task.h"
#include "bsp_ws2812_driver.h"
#include "bsp_battery_handler.h"
#include "robot_mode_service.h"
#include "robot_motion_cmd.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/
#define WS2812_TASK_PERIOD_MS            (50u)
#define TURN_SIGNAL_BLINK_TICKS          (5u)
#define TURN_SIGNAL_MIN_DIFF_RPM         (120)
#define TURN_SIGNAL_MIN_WHEEL_RPM        (80)
#define TURN_SIGNAL_COLOR                (0xFF8000u)
#define BATTERY_SOC_FULL_X100            (9000u)

/********************************** Variables ********************************/

typedef enum
{
    WS2812_STRIP_LEFT_FRONT = 0,
    WS2812_STRIP_RIGHT_FRONT,
    WS2812_STRIP_RIGHT_REAR,
    WS2812_STRIP_LEFT_REAR,
} ws2812_strip_pos_t;

typedef enum
{
    WS2812_TURN_NONE = 0,
    WS2812_TURN_LEFT,
    WS2812_TURN_RIGHT,
} ws2812_turn_dir_t;

typedef struct
{
    uint8_t first_led;
    uint8_t led_count;
} ws2812_led_range_t;


/********************************** Functions ********************************/
static ws2812_led_range_t ws2812_task_get_led_range(ws2812_strip_pos_t segment)
{
    ws2812_led_range_t range;

    switch (segment)
    {
        case WS2812_STRIP_LEFT_FRONT:
            range.first_led = (uint8_t)SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_FIRST_LED;
            range.led_count = (uint8_t)SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_LED_COUNT;
            break;
        case WS2812_STRIP_RIGHT_FRONT:
            range.first_led = (uint8_t)SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_FIRST_LED;
            range.led_count = (uint8_t)SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_LED_COUNT;
            break;
        case WS2812_STRIP_RIGHT_REAR:
            range.first_led = (uint8_t)SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_FIRST_LED;
            range.led_count = (uint8_t)SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_LED_COUNT;
            break;
        case WS2812_STRIP_LEFT_REAR:
        default:
            range.first_led = (uint8_t)SYSTEM_CFG_RGB_STRIP_LEFT_REAR_FIRST_LED;
            range.led_count = (uint8_t)SYSTEM_CFG_RGB_STRIP_LEFT_REAR_LED_COUNT;
            break;
    }

    return range;
}

static uint32_t ws2812_task_abs_i32(int32_t value)
{
    return (value >= 0) ? (uint32_t)value : (uint32_t)(-value);
}

static uint32_t ws2812_task_get_base_color(const ROBOT_MODE_T *robot_mode,
                                           uint8_t blink_on)
{
    BATTERY_INFO battery = {0};

    if (robot_mode == NULL)
    {
        return RGB_NONE;
    }

    if (robot_mode->robot_mode_main == MODE_EMERGENCY)
    {
        return (blink_on != 0u) ? RGB_RED : RGB_NONE;
    }

    if ((BatteryHandler_Snapshot(&battery) == 0u) &&
        (battery.comm_fault == 0u) &&
        ((battery.charge_status & 0x01u) != 0u))
    {
        return (battery.remain_capacity > BATTERY_SOC_FULL_X100) ? RGB_GREEN : RGB_RED;
    }

    if (robot_mode->robot_mode_main == MODE_READY)
    {
        return RGB_PURPLE;
    }

    if (robot_mode->robot_mode_main == MODE_REMOTE)
    {
        return (robot_mode->robot_mode_sub == SUB_SLAVE) ? RGB_BLUE : RGB_GREEN;
    }

    return RGB_NONE;
}

static void ws2812_task_set_segment_color(ws2812_strip_pos_t segment, uint32_t rgb)
{
    ws2812_led_range_t range = ws2812_task_get_led_range(segment);
    uint8_t start = range.first_led;
    uint8_t end = start + range.led_count;

    for (uint8_t i = start; (i < end) && (i < WS2811_LED_NUM); ++i)
    {
        ws2811_set_pixel(i, rgb);
    }
}

static ws2812_turn_dir_t ws2812_task_get_turn_dir(const ROBOT_MODE_T *robot_mode)
{
    robot_motion_cmd_t motion = {0};
    int32_t left_wheel_rpm;
    int32_t right_wheel_rpm;
    int32_t turn_delta_rpm;

    if (robot_mode == NULL)
    {
        return WS2812_TURN_NONE;
    }

    if ((robot_mode->robot_mode_main != MODE_REMOTE) ||
        ((robot_mode->robot_mode_sub != SUB_MANUAL) &&
         (robot_mode->robot_mode_sub != SUB_SLAVE)))
    {
        return WS2812_TURN_NONE;
    }

    robot_motion_cmd_snapshot(&motion);

    /* robot_motion_cmd_t stores the motor command sign. Convert it back to the
     * normalized wheel direction used by the host and telemetry path. */
    left_wheel_rpm = motion.left_rpm;
    right_wheel_rpm = -motion.right_rpm;

    if ((ws2812_task_abs_i32(left_wheel_rpm) < TURN_SIGNAL_MIN_WHEEL_RPM) &&
        (ws2812_task_abs_i32(right_wheel_rpm) < TURN_SIGNAL_MIN_WHEEL_RPM))
    {
        return WS2812_TURN_NONE;
    }

    /* Differential-drive turning direction:
     * right wheel faster -> left turn
     * left wheel faster  -> right turn */
    turn_delta_rpm = right_wheel_rpm - left_wheel_rpm;

    if (turn_delta_rpm >= TURN_SIGNAL_MIN_DIFF_RPM)
    {
        return WS2812_TURN_LEFT;
    }

    if (turn_delta_rpm <= -TURN_SIGNAL_MIN_DIFF_RPM)
    {
        return WS2812_TURN_RIGHT;
    }

    return WS2812_TURN_NONE;
}

static void ws2812_task_apply_turn_signal(ws2812_turn_dir_t turn_dir, uint8_t blink_on)
{
    if ((turn_dir == WS2812_TURN_NONE) || (blink_on == 0u))
    {
        return;
    }

    if (turn_dir == WS2812_TURN_LEFT)
    {
        ws2812_task_set_segment_color(WS2812_STRIP_LEFT_FRONT, TURN_SIGNAL_COLOR);
        ws2812_task_set_segment_color(WS2812_STRIP_LEFT_REAR, TURN_SIGNAL_COLOR);
        return;
    }

    ws2812_task_set_segment_color(WS2812_STRIP_RIGHT_FRONT, TURN_SIGNAL_COLOR);
    ws2812_task_set_segment_color(WS2812_STRIP_RIGHT_REAR, TURN_SIGNAL_COLOR);
}

void WS2812_Task(void *pvParameters)
{
	(void)pvParameters;

	TickType_t lastWakeTime = xTaskGetTickCount();
    ROBOT_MODE_T robot_mode;
    uint8_t blink_tick = 0u;
    uint8_t blink_on = 1u;

    while(1)
    {
		robot_mode_snapshot(&robot_mode);

        /* Logical positions are mapped through system_cfg.h LED ranges.
         * Default 8-LED mapping: [0..1]=LF, [2..3]=RF, [4..5]=RR, [6..7]=LR. */
        ws2811_all_same_color(ws2812_task_get_base_color(&robot_mode, blink_on));
        ws2812_task_apply_turn_signal(ws2812_task_get_turn_dir(&robot_mode), blink_on);

		ws2811_show();

        if (++blink_tick >= TURN_SIGNAL_BLINK_TICKS)
        {
            blink_tick = 0u;
            blink_on = (uint8_t)!blink_on;
        }

		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(WS2812_TASK_PERIOD_MS));
    }
}
