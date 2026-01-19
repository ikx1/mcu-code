/**
 ******************************************************************************
 * File Name          : display.c
 * Description        : 
 ******************************************************************************
 * @attention
 * 
 ******************************************************************************
 */

/********************************** Includes *********************************/
#include "bsp_handler_display.h"
#include "bsp_battery_handler.h"
#include "control_task.h"
#include "user_function.h"
#include "usart.h"

#include <string.h>

/********************************** Defines **********************************/


/********************************** Variables ********************************/
static DISPLAY_INFO display_info = {0};

/********************************** Functions ********************************/
DISPLAY_INFO * get_display_p(void)
{
	return &display_info;
}

void display_send(uint8_t data)
{
	while(!LL_USART_IsActiveFlag_TC(USART2));
	LL_USART_TransmitData8(USART2, data);
}

void display_send_str(uint8_t *send_str, uint8_t num)
{
	for(uint8_t i=0; i < num; i++)
	{
		display_send(send_str[i]);
	}	
}

uint8_t display_answer_read_data01(void)
{
	const BATTERY_INFO* battery_info = BatteryHandler_GetBatteryInfo();
	const ROBOT_MODE_T* robot_mode = get_robot_mode_p();

	static uint8_t  str[14] = {0};
	str[0] = 0xa0;
	str[1] = 0x0a;
	str[2] = 0x55;
	str[3] = 0x01;
	//	数据域开始
	str[4] = robot_mode->robot_mode_main;         // 机器人主状态
	str[5] =robot_mode->robot_mode_sub;			// 机器人子状态
	str[6] = battery_info->voltage[0];  //	电池电压高8，询问电池
	str[7] = battery_info->voltage[1];	//	电池电压低8，询问电池
	str[8] = battery_info->soc[0];	//	电池电量百分比
	str[9] = battery_info->soc[1];
	str[10] = battery_info->charge_status;  //0 放电 1 充电
	str[11] = battery_info->electricity[0]; 
	str[12] = battery_info->electricity[1]; 
	//	校验位
	str[13] = Serial_checksum(str, 13);
	//	数据发送
	display_send_str(str, 14);	
	return 0;
}

void display_callback(const uint8_t * data, uint8_t len)
{	
	if(data[2] == 0x55)//发送数据
	{
		display_answer_read_data01();
	}
	else if(data[2] == 0xaa) //接收数据
	{
		if(data[4] == 1)
		{
			display_info.moniter_state = 1; 	// 0-off 1-on

			switch(data[5])// 电机模式0-stop 1-mag 2-charge	
			{
				case 0x00:
					display_info.moniter_mode = 0;		//停止
					break;
				case 0x02:
					display_info.moniter_mode = 2;	//充电
					break;
				case 0x03:
					display_info.moniter_mode = 3;//停止充电
					break;
			}
		}
	}		
}
