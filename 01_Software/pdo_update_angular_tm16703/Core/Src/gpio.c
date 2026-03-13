/**
 * @file gpio.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "gpio.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/


void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);		// 使能PORTE,

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 			 	// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;		 		// IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);							// 初始化GPIOE
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	//PE7设置成上拉输入	
	GPIO_Init(GPIOE, &GPIO_InitStructure);					//初始化GPIOE	
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_ResetBits(GPIOE, GPIO_Pin_15 | GPIO_Pin_14); //将PE15置为低   一直在接收状态
}


