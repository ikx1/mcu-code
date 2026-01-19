/**
 * @file bsp_battery_driver.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_BATTERY_DRIVER_H__
#define __BSP_BATTERY_DRIVER_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Defines **********************************/
#define MODBUS_REC_BUF_SIZE 256
#define MODBUS_TIMEOUT_MS   5000  // 通信超时时间

#define MODBUS_HEADER_1        0xdd
#define MODBUS_HEADER_2        0x0d
#define MODBUS_TAIL			0x77
#define MODBUS_FRAME_LEN		51

/********************************** Variables ********************************/
typedef void (*modbus_frame_cb_t)(const uint8_t *frame, uint8_t len);

typedef enum
{
	MODBUS_IDLE = 0,
	MODBUS_RECEIVEING
}Modbus_Status_t;

typedef struct 
{
    uint8_t rec_buf[MODBUS_FRAME_LEN];
    uint16_t rec_index;
    uint8_t rec_idle_flag;
    uint32_t last_recv_time;      // 最后接收时间戳
    uint8_t comm_status;          // 通信状态 0-正常 1-断联
} ModbusRecData;

/********************************** Functions ********************************/
ModbusRecData* ModbusDriver_GetRecData(void);
uint8_t ModbusDriver_CheckCommStatus(void);
void MODBUS_USART_IRQHandler(void);
void ModbusDriver_UpdateTimestamp(uint32_t current_time);

void modbus_driver_register_callback(modbus_frame_cb_t cb);
void modbus_driver_input_byte(uint8_t byte);

#endif /* __BSP_BATTERY_DRIVER_H__ */
