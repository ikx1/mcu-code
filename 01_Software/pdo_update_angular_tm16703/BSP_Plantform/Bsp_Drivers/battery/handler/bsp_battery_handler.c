/**
 * @file bsp_battery_handler.c
 * @brief Battery data handler.
 */

/********************************** Includes *********************************/
#include "bsp_battery_handler.h"

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "system_cfg.h"

/********************************** Variables ********************************/
#ifndef DEBUG
static BATTERY_INFO battery = {0};
#else
BATTERY_INFO battery = {0};
#endif

/********************************** Functions ********************************/
const BATTERY_INFO* BatteryHandler_GetBatteryInfo(void)
{
    return &battery;
}

uint8_t BatteryHandler_Snapshot(BATTERY_INFO *out)
{
    if (out == NULL)
    {
        return 1u;
    }

    *out = battery;
    return 0u;
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

void modbus_callback(const uint8_t *data, uint8_t len)
{
    if (data == NULL)
    {
        return;
    }

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
    {
        battery.charge_status = 0;
    }
    else if(battery.run_status & 0x04)
    {
        battery.charge_status = 1;
    }

    battery.current_voltage = (battery.voltage[0] << 8 | battery.voltage[1]) * 0.01f;
    battery.current_i = (battery.electricity[0] << 8 | battery.electricity[1]) * 0.01f;
    battery.remain_capacity = battery.soc[0] << 8 | battery.soc[1];
    
    (void)len;
    battery.last_comm_time = xTaskGetTickCount();
    battery.comm_fault = 0;
}
