/**
 * @file delay.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __DELAY_H
#define __DELAY_H

/********************************** Includes *********************************/
#include "stm32f10x.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
void delay_init(void);
// 微秒级延时
void delay_us(uint32_t nus);
// 毫秒级延时
void delay_ms(uint32_t nms);

u32 getSysTickCnt(void);

#endif
