/**
 * @file bsp_display_driver.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_DISPLAY_DRIVER_H__
#define __BSP_DISPLAY_DRIVER_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Defines **********************************/
#define DISPLAY_FRAME    		14
#define DISPLAY_HEADER_1        0xa0
#define DISPLAY_HEADER_2        0x0a


/********************************** Variables ********************************/
typedef void (*display_frame_cb_t)(const uint8_t *frame, uint8_t len);

typedef enum 
{
    DISPLAY_IDLE,
    DISPLAY_RECEIVING
} display_rx_state_t;

typedef struct
{
	uint8_t buf[DISPLAY_FRAME];
	uint8_t state;
	uint8_t index;
}display_rec_t;

/********************************** Functions ********************************/
display_rec_t* display_getData_p(void);
void display_driver_register_callback(display_frame_cb_t cb);
void display_driver_input_byte(uint8_t byte);


#endif /* __BSP_DISPLAY_DRIVER_H__ */
