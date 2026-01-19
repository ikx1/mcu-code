/**
 * @file bsp_display_driver.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_display_driver.h"
#include "usart.h"
#include "user_function.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/
static display_rec_t display_data = {0};
static display_frame_cb_t frame_cb = NULL; 

/********************************** Functions ********************************/
display_rec_t* display_getData_p(void)
{
    return &display_data;
}

void display_driver_register_callback(display_frame_cb_t cb) 
{
    frame_cb = cb;
}

void display_driver_input_byte(uint8_t byte) 
{
    switch (display_data.state) 
    {
        case DISPLAY_IDLE:
            if (byte == DISPLAY_HEADER_1) 
            {
                display_data.index = 0;
                display_data.buf[display_data.index++] = byte;
                display_data.state = DISPLAY_RECEIVING;
            }
            break;

        case DISPLAY_RECEIVING:
            if (display_data.index < DISPLAY_FRAME) 
            {
                display_data.buf[display_data.index++] = byte;

                if (display_data.index == DISPLAY_FRAME) 
                {
                    display_data.state = DISPLAY_IDLE;

					uint8_t sum = Serial_checksum(display_data.buf, 13);
                    if (sum == display_data.buf[13] && frame_cb) 
                    {
                        frame_cb(display_data.buf, DISPLAY_FRAME);
                    }
                }
            } 
            else 
            {
                display_data.state = DISPLAY_IDLE;
            }
            break;

        default:
            display_data.state = DISPLAY_IDLE;
            break;
    }
}

