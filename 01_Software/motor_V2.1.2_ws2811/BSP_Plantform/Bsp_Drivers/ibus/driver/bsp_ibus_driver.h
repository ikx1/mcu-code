/**
 * @file bsp_ibus_driver.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_IBUS_DRIVER_H__
#define __BSP_IBUS_DRIVER_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include <stdio.h>

/********************************** Defines **********************************/
#define IBUS_CHANNEL_COUNT   14

#define IBUS_FRAME_LENGTH    32
#define IBUS_HEADER_1        0x20
#define IBUS_HEADER_2        0x40

/********************************** Variables ********************************/
typedef void (*ibus_frame_cb_t)(const uint8_t *frame, uint8_t len);

typedef enum 
{
    IBUS_RX_IDLE,
    IBUS_RX_RECEIVING
} ibus_rx_state_t;


/********************************** Functions ********************************/
/**
 * @brief 初始化 ibus 驱动
 */
void ibus_driver_init(void);

/**
 * @brief 注册帧处理回调函数
 */
void ibus_driver_register_callback(ibus_frame_cb_t cb);

/**
 * @brief 提供一个字节给 ibus 驱动（串口接收中断中调用）
 */
void ibus_driver_input_byte(uint8_t byte);

#endif /* __BSP_IBUS_H__ */
