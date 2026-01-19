/**
 * @file bsp_ultrasonic_driver.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "bsp_ultrasonic_driver.h"


#define ULTRA_TX_FRAME_LEN 8
const uint8_t ultra_tx_buf[ULTRA_TX_FRAME_LEN] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0x85, 0xF6};

bsp_ultrasonic_status_t ultrasonic_driver_read_distance(uint16_t *distance_mm) 
{
//    uint8_t rx_buf[7] = {0};
    
//    ultrasonic_rs485_send(ultra_tx_buf, ULTRA_TX_FRAME_LEN);
//    ultrasonic_delay_ms(10);
//    
//    if (!ultrasonic_rs485_recv(rx_buf, 7, 100)) return false;

//    if (rx_buf[0] == 0x01 && rx_buf[1] == 0x03 && rx_buf[2] == 0x02) 
//    {
//        *distance_mm = (rx_buf[3] << 8) | rx_buf[4];
//        return ULTRASONIC_OK;
//    }

    return ULTRASONIC_ERROR;
}
