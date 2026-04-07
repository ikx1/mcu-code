/**
 * @file bsp_ultrasonic_driver.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_ULTRASONIC_DRIVER_H__
#define __BSP_ULTRASONIC_DRIVER_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Defines **********************************/
#define ULTRASONIC_MODBUS_ADDR        0x01u
#define ULTRASONIC_MODBUS_FUNC        0x03u
#define ULTRASONIC_MODBUS_BYTE_COUNT  0x02u

#define ULTRASONIC_TX_FRAME_LEN       8u
#define ULTRASONIC_RX_FRAME_LEN       7u
#define ULTRASONIC_POLL_TIMEOUT_MS    100u
#define ULTRASONIC_COMM_TIMEOUT_MS    500u

/********************************** Variables ********************************/
typedef enum
{
    ULTRASONIC_OK = 0,
    ULTRASONIC_ERROR,
    ULTRASONIC_TIMEOUT,
    ULTRASONIC_CRC_ERROR,
    ULTRASONIC_FRAME_ERROR,
    ULTRASONIC_INVALID_PARAM,
} bsp_ultrasonic_status_t;

typedef struct
{
    uint16_t distance_mm;
    uint32_t last_recv_time;
    uint8_t comm_status;
} UltrasonicRecData;

/********************************** Functions ********************************/
void ultrasonic_driver_init(void);
const UltrasonicRecData* ultrasonic_driver_get_data(void);
uint8_t ultrasonic_driver_check_comm_status(void);
bsp_ultrasonic_status_t ultrasonic_driver_read_distance(uint16_t *distance_mm);


#endif //__BSP_ULTRASONIC_DRIVER_H__
