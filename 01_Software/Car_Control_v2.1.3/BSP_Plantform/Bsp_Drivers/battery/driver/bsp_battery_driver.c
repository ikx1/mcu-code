/**
 * @file bsp_battery_driver.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_battery_driver.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
static ModbusRecData rec_data = {0};
//ModbusRecData rec_data = {0};
static modbus_frame_cb_t frame_cb = NULL; // 用户注册的帧处理回调


/********************************** Functions ********************************/
ModbusRecData* ModbusDriver_GetRecData(void)
{
    return &rec_data;  // 返回静态全局变量rec_data的指针
}

/**
 * @brief 
 * 
 * @param current_time 
 */
void ModbusDriver_UpdateTimestamp(uint32_t current_time)
{
    rec_data.last_recv_time = current_time;
    rec_data.comm_status = 0; // 收到数据时重置通信状态为正常
}

/**
 * @brief 
 * 
 * @return uint8_t 
 */
uint8_t ModbusDriver_CheckCommStatus(void)
{
    uint32_t current_time = xTaskGetTickCount();
    
    // 检查是否超时
    if((current_time - rec_data.last_recv_time) > pdMS_TO_TICKS(MODBUS_TIMEOUT_MS))
    {
        rec_data.comm_status = 1; // 标记为断联
    }
    
    return rec_data.comm_status;
}

/**
 * @brief 用户注册帧处理回调函数
 */
void modbus_driver_register_callback(modbus_frame_cb_t cb) 
{
    frame_cb = cb;
}


void modbus_driver_input_byte(uint8_t byte) 
{
    switch (rec_data.rec_idle_flag) 
    {
        case MODBUS_IDLE:
            if (byte == MODBUS_HEADER_1) 
            {
				rec_data.rec_index = 0;
				rec_data.rec_buf[rec_data.rec_index++] = byte;
				rec_data.rec_idle_flag = MODBUS_RECEIVEING;
            }
            break;

        case MODBUS_RECEIVEING:
            if (rec_data.rec_index < MODBUS_FRAME_LEN) 
            {
				rec_data.rec_buf[rec_data.rec_index++] = byte;

                if (rec_data.rec_index == MODBUS_FRAME_LEN) 
                {
                    rec_data.rec_idle_flag = MODBUS_IDLE;

					uint8_t head = rec_data.rec_buf[1];
					uint8_t tail = rec_data.rec_buf[rec_data.rec_index-1];
					
                    if (head == MODBUS_HEADER_2 && tail == MODBUS_TAIL && frame_cb) 
                    {
                        frame_cb(rec_data.rec_buf, MODBUS_FRAME_LEN);
                    }
                }
            } 
            else 
            {
                rec_data.rec_idle_flag = MODBUS_IDLE;
            }
            break;

        default:
			memset(rec_data.rec_buf, 0, sizeof(rec_data.rec_buf));
            break;
    }	
}
