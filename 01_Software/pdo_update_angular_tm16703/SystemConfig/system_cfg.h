/**
 * @file system_cfg.h
 * @author your name (you@domain.com)
 * @brief 系统配置头文件
 * @note System-wide configuration header for geometry, IDs, and runtime policy.
 * @version 0.1
 * @date 2023-08-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __SYSTEM_CFG_H__
#define __SYSTEM_CFG_H__

#include <stdio.h>
#include <stdint.h>
/* Product variant selector for the facility platform build. */
#define Version_facility	

/********************************** Parameter ********************************/
/* PI */
/* Circular constant used by geometry and speed conversions. */
#define PI										    (3.141592654f)

/*---------------------------------------------------------------------------*/
#ifdef Version_facility
	/* Facility platform chassis dimensions. */
	/* robot wheelbase(width) */
    /* Distance between the left and right wheel centers. */
	#define WHEEL_WIDTH								0.40f
    /* robot radius */
    /* Radius of the drive wheel used by linear-speed conversion. */
    #define WHEEL_RADIUS						    (0.085f)
    /* Wheel diameter */
    /* Diameter of the drive wheel. */
    #define WHEEL_DIAMETER							0.175f	
#endif /* Version2_0_CAR_LEN */
/*---------------------------------------------------------------------------*/
/* Wheel perimeter */
/* Drive wheel perimeter in meters. */
#define WHEEL_PERIMETER							(WHEEL_DIAMETER * PI)
/*---------------------------------------------------------------------------*/
/* motor encoder resolution */
/* Motor encoder counts per motor revolution. */
#define MOTOR_POS								5600.0f
/* Mechanical reduction ratio from motor shaft to wheel output. */
#define DRIVE_GEAR_RATIO                        (10.0f)
#ifndef WHEEL_REDUCTION_RATIO
#define WHEEL_REDUCTION_RATIO                   DRIVE_GEAR_RATIO
#endif

/*---------------------------------------------------------------------------*/
/* degree of full turn */
/* Degrees in one complete rotation. */
#define DEGREE_FUL_TURN							360.0f
/* The encoder value for one turn of the motor */
/* Encoder counts represented by one degree of motor rotation. */
#define MOTOR_1_DEGREE_TURN         		(DRIVE_GEAR_RATIO * MOTOR_POS / DEGREE_FUL_TURN)

/*---------------------------------------------------------------------------*/
/* 180° */
/* Degrees in half a rotation. */
#define DEGREE_180								180.0f
/* Degrees in half a turn. */
/* Radian to angle */
/* Conversion factor from radians to degrees. */
#define CONVERT_RAD_TO_DEGREE					(DEGREE_180 / PI)
/* Radian-to-degree conversion factor. */
/* Angle to radian */
/* Conversion factor from degrees to radians. */
#define DEGREE_TO_CONVERT_RAD					(PI / DEGREE_180)
/* Degree-to-radian conversion factor. */
/* One minute */
/* Seconds per minute, used by RPM conversions. */
#define MIN										(60.0f)
/* half of minute */
/* Legacy half-minute constant retained for existing formulas. */
#define HALF_MINUTE								(MIN / 2.0f)

/* Convert between RPM and angular speed in rad/s. */
#define RPM_TO_ANGULAR(rpm)   ((rpm) * (2.0f * PI) / MIN)
/* Convert angular speed in rad/s back to RPM. */
#define ANGULAR_TO_RPM(w)     ((w)   * MIN / (2.0f * PI))

/* Generic floating-point tolerance. */
#define EPSILON              0.001f

/* RGB strip logical LED count. Update this value when the physical strip
 * length changes instead of editing the driver header. */
#ifndef SYSTEM_CFG_RGB_STRIP_LED_NUM
#define SYSTEM_CFG_RGB_STRIP_LED_NUM            (4u)
#endif

/* RGB strip segment-position mapping:
 * The strip is handled as 4 logical positions: left-front, right-front,
 * right-rear, left-rear. Each macro maps one logical position to the physical
 * segment index along the strip data direction.
 * Default 0,1,2,3 matches: [LF][RF][RR][LR]
 * Reverse installation can use: [3][2][1][0]
 */
#ifndef SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT
#define SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT   (0u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT
#define SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT  (1u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR
#define SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR   (2u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR
#define SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR    (3u)
#endif

#if ((SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT > 3u) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT > 3u) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR > 3u) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR > 3u))
#error "RGB strip segment mapping must stay within [0..3]"
#endif

#if ((SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT == SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT == SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_FRONT == SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT == SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_FRONT == SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR) || \
     (SYSTEM_CFG_RGB_STRIP_SEGMENT_RIGHT_REAR == SYSTEM_CFG_RGB_STRIP_SEGMENT_LEFT_REAR))
#error "RGB strip segment mapping must use four unique segment indexes"
#endif

/* RGB strip logical-position LED ranges.
 * Use these macros when the four corners do not have equal LED counts or when
 * the physical start LED is not aligned with the default 2-LED segments.
 * The current default matches an 8-LED strip:
 * [0..1]=LF, [2..3]=RF, [4..5]=RR, [6..7]=LR
 */
#ifndef SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_FIRST_LED
#define SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_FIRST_LED   (0u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_LED_COUNT
#define SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_LED_COUNT   (1u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_FIRST_LED
#define SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_FIRST_LED  (1u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_LED_COUNT
#define SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_LED_COUNT  (1u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_FIRST_LED
#define SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_FIRST_LED   (2u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_LED_COUNT
#define SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_LED_COUNT   (1u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_LEFT_REAR_FIRST_LED
#define SYSTEM_CFG_RGB_STRIP_LEFT_REAR_FIRST_LED    (3u)
#endif

#ifndef SYSTEM_CFG_RGB_STRIP_LEFT_REAR_LED_COUNT
#define SYSTEM_CFG_RGB_STRIP_LEFT_REAR_LED_COUNT    (1u)
#endif

#if (((SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_FIRST_LED + SYSTEM_CFG_RGB_STRIP_LEFT_FRONT_LED_COUNT) > SYSTEM_CFG_RGB_STRIP_LED_NUM) || \
     ((SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_FIRST_LED + SYSTEM_CFG_RGB_STRIP_RIGHT_FRONT_LED_COUNT) > SYSTEM_CFG_RGB_STRIP_LED_NUM) || \
     ((SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_FIRST_LED + SYSTEM_CFG_RGB_STRIP_RIGHT_REAR_LED_COUNT) > SYSTEM_CFG_RGB_STRIP_LED_NUM) || \
     ((SYSTEM_CFG_RGB_STRIP_LEFT_REAR_FIRST_LED + SYSTEM_CFG_RGB_STRIP_LEFT_REAR_LED_COUNT) > SYSTEM_CFG_RGB_STRIP_LED_NUM))
#error "RGB strip logical-position LED range exceeds SYSTEM_CFG_RGB_STRIP_LED_NUM"
#endif
/********************************** CAN ID ***********************************/
/* CANopen node IDs for right wheel, left wheel, and joint drives. */
/* Node ID assigned to the right wheel drive. */
#define  	Right_Wheel_ID                0x0001
/* Node ID assigned to the left wheel drive. */
#define  	Left_Wheel_ID                 0x0002
/* Node ID assigned to the joint / lift drive. */
#define  	Joint_Wheel_ID                0x0003

/* Mechanical layout and steering reference for the numbered wheel positions below. */
/**
 * 驱动
 *  3		2
 *  4		1
 * 
 * 转向
 * 	6		7
 * 	5		8
 */

/*---------------------------------------------------------------------------*/
/* Non-production debug hooks are opt-in; keep disabled by default. */
#ifndef SYSTEM_CFG_ENABLE_DEBUG
/* Default debug-hook switch. */
#define SYSTEM_CFG_ENABLE_DEBUG                    (1u)
#endif

#if (SYSTEM_CFG_ENABLE_DEBUG != 0u)
#ifndef DEBUG
/* Mirror the generic DEBUG macro when debug hooks are enabled. */
#define DEBUG
#endif
#endif

/*---------------------------------------------------------------------------*/
/* READY mode drive power policy:
 * 0: safe standby, drive motors stay disabled in MODE_READY
 * 1: powered standby, drive motors may stay enabled in MODE_READY
 */
#ifndef SYSTEM_CFG_READY_KEEP_DRIVE_POWER
/* Default READY-mode drive power policy. */
#define SYSTEM_CFG_READY_KEEP_DRIVE_POWER          (1u)
#endif

/*---------------------------------------------------------------------------*/
/* Host takeover policy without IBUS:
 * 0: no-IBUS means the robot stays in MODE_READY
 * 1: fresh host drive/lift traffic may force MODE_REMOTE + SUB_SLAVE even
 *    when the remote receiver is disconnected
 */
#ifndef SYSTEM_CFG_ALLOW_HOST_SLAVE_WITHOUT_IBUS
/* Default no-IBUS host takeover policy for the current F103 platform. */
#define SYSTEM_CFG_ALLOW_HOST_SLAVE_WITHOUT_IBUS   (1u)
#endif

/*---------------------------------------------------------------------------*/
/* Host navigation drive-power hold policy:
 * 0: SUB_SLAVE still requires a live host session to keep drive power enabled
 * 1: once the robot has entered MODE_REMOTE + SUB_SLAVE, drive motors stay
 *    enabled until mode/emergency/explicit fresh disable says otherwise
 */
#ifndef SYSTEM_CFG_HOST_SLAVE_KEEP_DRIVE_POWER
/* Default host-navigation power-hold policy for the current platform. */
#define SYSTEM_CFG_HOST_SLAVE_KEEP_DRIVE_POWER     (1u)
#endif

/*---------------------------------------------------------------------------*/
/* Joint homing policy at power-up:
 * 0: skip auto homing and treat the slide axis as already homed
 * 1: auto home the slide axis after power-up
 *
 * Absolute-encoder slide motors can set this to 0 to avoid re-homing.
 */
#ifndef SYSTEM_CFG_JOINT_AUTO_HOME_ON_BOOT
/* Default power-up homing policy for the joint / lift axis. */
#define SYSTEM_CFG_JOINT_AUTO_HOME_ON_BOOT         (1u)
#endif

/*---------------------------------------------------------------------------*/
/* Joint travel and position-unit scaling:
 * - SYSTEM_CFG_JOINT_MAX_TRAVEL_MM bounds host absolute lift commands
 * - SYSTEM_CFG_JOINT_POS_PER_UNIT / SYSTEM_CFG_JOINT_MM_PER_UNIT define the
 *   conversion between motor position units and millimeters
 * - SYSTEM_CFG_JOINT_MANUAL_STEP_MAX controls one full-stick manual jog step
 * - SYSTEM_CFG_JOINT_HOME_NEG_TARGET_POS_CMD is the relative move used when
 *   seeking the lower limit during homing
 * - SYSTEM_CFG_JOINT_HOME_TIMEOUT_MS is estimated from travel length,
 *   homing-speed estimate and timeout margin
 * - SYSTEM_CFG_JOINT_HOME_SETTLE_MS is the debounce/settle delay after the
 *   lower limit is hit and before reset_zero executes
 */
/* Maximum allowed lift travel in millimeters for host absolute-position commands. */
#ifndef SYSTEM_CFG_JOINT_MAX_TRAVEL_MM
#define SYSTEM_CFG_JOINT_MAX_TRAVEL_MM            (700)
#endif

/* Motor position units represented by SYSTEM_CFG_JOINT_MM_PER_UNIT millimeters. */
#ifndef SYSTEM_CFG_JOINT_POS_PER_UNIT
#define SYSTEM_CFG_JOINT_POS_PER_UNIT             (10000)
#endif

/* Millimeter travel represented by SYSTEM_CFG_JOINT_POS_PER_UNIT units. */
#ifndef SYSTEM_CFG_JOINT_MM_PER_UNIT
#define SYSTEM_CFG_JOINT_MM_PER_UNIT              (5)
#endif

/* Maximum manual jog step for one full-scale operator command. */
#ifndef SYSTEM_CFG_JOINT_MANUAL_STEP_MAX
#define SYSTEM_CFG_JOINT_MANUAL_STEP_MAX          (10000)
#endif

/* Relative negative move command used while seeking the lower limit during homing. */
#ifndef SYSTEM_CFG_JOINT_HOME_NEG_TARGET_POS_CMD
#define SYSTEM_CFG_JOINT_HOME_NEG_TARGET_POS_CMD  (-(int32_t)10000)
#endif

/* Estimated homing speed in millimeters per second. */
#ifndef SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S
#define SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S  (20u)
#endif

/* Extra timeout margin added on top of the theoretical homing duration. */
#ifndef SYSTEM_CFG_JOINT_HOME_TIMEOUT_MARGIN_MS
#define SYSTEM_CFG_JOINT_HOME_TIMEOUT_MARGIN_MS   (4000u)
#endif

/* Settling delay after the lower limit is hit before zero-reset executes. */
#ifndef SYSTEM_CFG_JOINT_HOME_SETTLE_MS
#define SYSTEM_CFG_JOINT_HOME_SETTLE_MS           (300u)
#endif

#if (SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S == 0u)
#error "SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S must be non-zero"
#endif

/* Derived joint homing timeout in milliseconds from travel, speed, and margin. */
#define SYSTEM_CFG_JOINT_HOME_TIMEOUT_MS \
    ((((uint32_t)(SYSTEM_CFG_JOINT_MAX_TRAVEL_MM) * 1000u) + \
      ((uint32_t)(SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S) - 1u)) / \
     (uint32_t)(SYSTEM_CFG_JOINT_HOME_EST_SPEED_MM_PER_S) + \
     (uint32_t)(SYSTEM_CFG_JOINT_HOME_TIMEOUT_MARGIN_MS))

/* Convert lift travel in millimeters to the motor position-unit domain. */
#define SYSTEM_CFG_JOINT_MM_TO_POS(mm) \
    ((int32_t)(((float)(mm) * (float)SYSTEM_CFG_JOINT_POS_PER_UNIT) / \
               (float)SYSTEM_CFG_JOINT_MM_PER_UNIT))

/* Convert motor position units back to lift travel in millimeters. */
#define SYSTEM_CFG_JOINT_POS_TO_MM(pos) \
    (((float)(pos) * (float)SYSTEM_CFG_JOINT_MM_PER_UNIT) / \
     (float)SYSTEM_CFG_JOINT_POS_PER_UNIT)

/*---------------------------------------------------------------------------*/
/* CAN diagnostics telemetry policy */
/* RX queue depth threshold that raises a telemetry warning. */
#ifndef CAN_DIAG_RX_DEPTH_WARN
#define CAN_DIAG_RX_DEPTH_WARN                 (24u)
#endif

/* Hold time for reporting RX-drop status after a drop event. */
#ifndef CAN_DIAG_RX_DROP_HOLD_MS
#define CAN_DIAG_RX_DROP_HOLD_MS               (3000u)
#endif

/* Hold time for reporting CAN bus-off state after detection. */
#ifndef CAN_DIAG_BUSOFF_HOLD_MS
#define CAN_DIAG_BUSOFF_HOLD_MS                (5000u)
#endif

/* Hold time for reporting CAN recovery state after a recovery event. */
#ifndef CAN_DIAG_RECOVER_HOLD_MS
#define CAN_DIAG_RECOVER_HOLD_MS               (5000u)
#endif

/* Period for automatically clearing accumulated CAN diagnostic counters. */
#ifndef CAN_DIAG_COUNTER_AUTO_CLEAR_MS
#define CAN_DIAG_COUNTER_AUTO_CLEAR_MS         (60000u)
#endif

#endif // __SYSTEM_CFG_H__
