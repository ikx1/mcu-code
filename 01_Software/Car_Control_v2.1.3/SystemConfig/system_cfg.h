/**
 * @file system_cfg.h
 * @author your name (you@domain.com)
 * @brief 系统配置头文件
 * @version 0.1
 * @date 2023-08-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __SYSTEM_CFG_H__
#define __SYSTEM_CFG_H__

#include <stdio.h>

#define Version_facility	

/********************************** Parameter ********************************/
/* PI */
#define PI										    3.1415926f

/*---------------------------------------------------------------------------*/
#ifdef Version_facility
	/* robot wheelbase(width) */
	#define WHEEL_WIDTH								0.375f
    /* Wheel diameter */
    #define WHEEL_DIAMETER							0.17f	
#endif /* Version2_0_CAR_LEN */
/*---------------------------------------------------------------------------*/
/* Wheel perimeter */
#define WHEEL_PERIMETER							(WHEEL_DIAMETER * PI)
/*---------------------------------------------------------------------------*/
/* motor encoder resolution */
#define MOTOR_POS								5600.0f
/*---------------------------------------------------------------------------*/
/* degree of full turn */
#define DEGREE_FUL_TURN							360.0f
/* The encoder value for one turn of the motor */
#define MOTOR_1_DEGREE_TURN         		(WHEEL_REDUCTION_RATIO * MOTOR_POS / DEGREE_FUL_TURN)
/*---------------------------------------------------------------------------*/
/* 180° */
#define DEGREE_180								180.0f
/* Radian to angle */
#define CONVERT_RAD_TO_DEGREE					(DEGREE_180 / PI)
/* Angle to radian */
#define DEGREE_TO_CONVERT_RAD					(PI / DEGREE_180)
/* One minute */
#define MIN										60

#define EPSILON              0.001f
/********************************** CAN ID ***********************************/
#define  	Right_Wheel_ID                0x0001
#define  	Left_Wheel_ID                 0x0002
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
/* enable navigation debug */
#define DEBUG

#endif // __SYSTEM_CFG_H__
