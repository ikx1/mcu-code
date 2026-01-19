/**
 * @file bsp_battery_handler.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_BATTERY_HANDLER_H__
#define __BSP_BATTERY_HANDLER_H__

/********************************** Includes *********************************/
#include "bsp_battery_driver.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef struct 
{ 
	uint8_t voltage[2];
	uint8_t electricity[2];
	uint8_t soc[2];

	uint16_t  remain_capacity;  	//soc百分比   剩余电量百分比(10000 倍）
	float  		current_i;			//实时电流 单位10ma
	float  		current_voltage;	//实时总电压 10mv
	uint16_t  temperature;			//温度
	uint16_t 	cycle_index;		//循环次数
	uint16_t protection_status;		//保护状态
	uint8_t run_status;			//运行状态
	uint8_t charge_status;		//充电状态	  

    uint8_t comm_fault;          	// 通信故障标志
    uint32_t last_comm_time;     	// 最后通信成功时间
}BATTERY_INFO;


/********************************** Functions ********************************/
const BATTERY_INFO* BatteryHandler_GetBatteryInfo(void);

void BatteryHandler_Init(void);
void modbus_callback(const uint8_t * data, uint8_t len);
uint8_t BatteryHandler_CheckConnection(void);

#endif /* __BSP_BATTERY_HANDLER_H__ */

