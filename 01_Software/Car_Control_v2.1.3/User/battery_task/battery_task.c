/**
 ******************************************************************************
 * File Name          : battery.c
 * Description        : 
 ******************************************************************************
 * @attention
 * 
 ******************************************************************************
 */

/********************************** Includes *********************************/
#include "battery_task.h"
#include "bsp_battery_handler.h"
#include "main.h"
#include "bsp_uart_driver.h"

#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
// 实现检查连接函数
void ModbusTask_CheckConnection(void)
{
    if(BatteryHandler_CheckConnection())
    {
        
    }

}

void Modbus_SendStr(uint8_t data)
{
	while(!LL_USART_IsActiveFlag_TC(USART3));
	HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, GPIO_PIN_SET);
	LL_USART_TransmitData8(USART3, data);
	while(!LL_USART_IsActiveFlag_TC(USART3));
	HAL_GPIO_WritePin(RS485_1_RW_GPIO_Port, RS485_1_RW_Pin, GPIO_PIN_RESET);
}

void Modbus_SendBuf(uint8_t *str, uint8_t len)
{
	for(uint8_t i = 0; i < len; i ++)
		Modbus_SendStr(str[i]);
}


uint8_t BatteryHandler_SendVoltageQuery(void)
{
    static uint8_t str[9] = {
        0xdd, 0x0d, 0x03, 0x03, 0x01, 0x00, 0x15, 0xf8, 0x77};
	Modbus_SendBuf(str, sizeof(str));
		
    return 0;
}

void ModbusTask(void *pvParameters)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	
    while(1)
    {
        BatteryHandler_SendVoltageQuery();
        
        // 检查连接状态
        ModbusTask_CheckConnection();
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
    }
}
