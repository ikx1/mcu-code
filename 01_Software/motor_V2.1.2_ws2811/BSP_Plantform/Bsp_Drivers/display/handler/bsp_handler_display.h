#ifndef __BSP_HANDLER_DISPLAY_H__
#define __BSP_HANDLER_DISPLAY_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include "bsp_display_driver.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
typedef struct
{
	uint8_t moniter_state; 	// 0-off 1-on
	uint8_t moniter_mode; 	//  0-stop  2-charge 3-stop_charge
	uint8_t uart_connect_flag;
}DISPLAY_INFO;

/********************************** Functions ********************************/
uint8_t display_handler_snapshot(DISPLAY_INFO *out);
void display_handler_set_robot_mode(uint8_t main_mode, uint8_t sub_mode);
void display_handler_poll(void);
void display_callback(const uint8_t * data, uint8_t len);


#endif /* __BSP_HANDLER_DISPLAY_H__ */
