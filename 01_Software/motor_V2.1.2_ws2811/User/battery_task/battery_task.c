#include "battery_task.h"

#include "bsp_battery_driver.h"
#include "bsp_battery_handler.h"

#include "FreeRTOS.h"
#include "task.h"

static void ModbusTask_CheckConnection(void)
{
    (void)BatteryHandler_CheckConnection();
}

void ModbusTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        modbus_driver_send_voltage_query();
        ModbusTask_CheckConnection();

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
