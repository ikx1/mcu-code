/**
 * @file bsp_battery_handler.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_battery_handler.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
//static BATTERY_INFO battery = {0};
BATTERY_INFO battery = {0};

/********************************** Functions ********************************/
const BATTERY_INFO* BatteryHandler_GetBatteryInfo(void)
{
    return &battery;
}

void BatteryHandler_Init(void)
{
    memset(&battery, 0, sizeof(BATTERY_INFO));
}

uint8_t BatteryHandler_CheckConnection(void)
{
    uint8_t driver_status = ModbusDriver_CheckCommStatus();
    
    if(driver_status)
    {
        battery.comm_fault = 1;
    }
    else
    {
        // 额外检查电池数据是否长时间未更新
        uint32_t current_time = xTaskGetTickCount();
        if((current_time - battery.last_comm_time) > pdMS_TO_TICKS(MODBUS_TIMEOUT_MS * 2))
        {
            battery.comm_fault = 1;
        }
        else
        {
            battery.comm_fault = 0;
        }
    }
    
    return battery.comm_fault;
}

void modbus_callback(const uint8_t * data, uint8_t len)
{
	battery.voltage[0] = data[12];
	battery.voltage[1] = data[13];    
	battery.electricity[0] = data[14];
	battery.electricity[1] = data[15];        
	battery.soc[0] = data[18];
	battery.soc[1] = data[19];    
	battery.cycle_index = data[20] << 8 | data[21];
    battery.protection_status = data[27] << 8 | data[28];
	battery.run_status = data[29];

	if(battery.run_status & 0x08)
		battery.charge_status = 0;
	else if(battery.run_status & 0x04)
		battery.charge_status = 1;    

	battery.current_voltage = (battery.voltage[0] << 8 | battery.voltage[1]) * 0.01f;
	battery.current_i = (battery.electricity[0] << 8 | battery.electricity[1]) * 0.01f;
	battery.remain_capacity = battery.soc[0] << 8 | battery.soc[1];
	
	// 更新最后通信时间
	battery.last_comm_time = xTaskGetTickCount();
	battery.comm_fault = 0;        
}

