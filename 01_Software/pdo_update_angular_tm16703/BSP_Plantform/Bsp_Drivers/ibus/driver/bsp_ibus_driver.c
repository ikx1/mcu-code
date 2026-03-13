/**
 * @file bsp_ibus_driver.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_ibus_driver.h"

#include <stddef.h>
#include <string.h>

/********************************** Defines **********************************/


/********************************** Variables ********************************/
static struct 
{
    uint8_t  buf[IBUS_FRAME_LENGTH];
    uint8_t  index;
    ibus_rx_state_t state;
}ibus_rx;

static ibus_frame_cb_t frame_cb = NULL; // 用户注册的帧处理回调


/********************************** Functions ********************************/
/**
 * @brief 初始化 ibus 驱动
 */
void ibus_driver_init(void) 
{
    memset(&ibus_rx, 0, sizeof(ibus_rx));
    ibus_rx.state = IBUS_RX_IDLE;
}

/**
 * @brief 用户注册帧处理回调函数
 */
void ibus_driver_register_callback(ibus_frame_cb_t cb) 
{
    frame_cb = cb;
}

/**
 * @brief CRC 校验（iBus: sum 校验）
 */
static uint16_t ibus_checksum(const uint8_t *data) 
{
    uint16_t sum = 0xFFFF - data[0] - data[1];
    
    for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) 
	{
        sum -= (data[3 + i*2] + data[2 + i*2]);
    }

    return sum;
}

/**
 * @brief 接收一个字节（由串口接收中断中调用）
 */
void ibus_driver_input_byte(uint8_t byte) 
{
    switch (ibus_rx.state) 
    {
        case IBUS_RX_IDLE:
            if (byte == IBUS_HEADER_1) 
            {
                ibus_rx.index = 0;
                ibus_rx.buf[ibus_rx.index++] = byte;
                ibus_rx.state = IBUS_RX_RECEIVING;
            }
            break;

        case IBUS_RX_RECEIVING:
            if (ibus_rx.index < IBUS_FRAME_LENGTH) 
            {
                ibus_rx.buf[ibus_rx.index++] = byte;

                if (ibus_rx.index == IBUS_FRAME_LENGTH) 
                {
                    ibus_rx.state = IBUS_RX_IDLE;

                    // 校验
                    uint16_t sum = ibus_checksum(ibus_rx.buf);
                    uint16_t recv = ibus_rx.buf[IBUS_FRAME_LENGTH - 2] | (ibus_rx.buf[IBUS_FRAME_LENGTH - 1] << 8);

                    if (sum == recv && frame_cb) 
                    {
                        frame_cb(ibus_rx.buf, IBUS_FRAME_LENGTH);
                    }
                }
            } 
            else 
            {
                ibus_rx.state = IBUS_RX_IDLE;
            }
            break;

        default:
            ibus_rx.state = IBUS_RX_IDLE;
            break;
    }
}
