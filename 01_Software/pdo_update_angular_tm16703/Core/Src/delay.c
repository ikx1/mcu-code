/**
 * @file delay.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "delay.h"

#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
static uint32_t fac_us = 0; // 微秒延时基数
static uint32_t fac_ms = 0; // 毫秒延时基数


/********************************** Functions ********************************/
// 初始化延时函数
void delay_init(void) 
{
    uint32_t reload;

    // 选择SysTick时钟源为HCLK（72MHz）
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    // 计算微秒延时基数
    fac_us = SystemCoreClock / 1000000; // 72MHz / 1,000,000 = 72

    reload = SystemCoreClock / configTICK_RATE_HZ;
    fac_ms = 1000 / configTICK_RATE_HZ;

    SysTick->LOAD = reload - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// 微秒延时
void delay_us(uint32_t nus) 
{
    uint32_t ticks = nus * fac_us;
    uint32_t told = SysTick->VAL;
    uint32_t tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;

    while (1) 
    {
        tnow = SysTick->VAL;
        if (tnow != told) 
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
                
            told = tnow;
            if (tcnt >= ticks)
                break;
        }
    }
}
// 毫秒延时
void delay_ms(uint32_t nms) 
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) 
    {
        vTaskDelay(nms / fac_ms);
        return;
    }

    while (nms--) delay_us(1000);
}

/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) 
	{
		// 如果调度器已经启动，返回FreeRTOS的系统节拍计数
		return xTaskGetTickCount();
	} 
	else 
	{
		// 如果调度器未启动，返回SysTick计数器的值
		static u32 tickCount = 0;
		static u32 lastSysTickVal = 0;
		u32 currentSysTickVal = SysTick->VAL;

		if (currentSysTickVal < lastSysTickVal) {
			// 如果SysTick计数器溢出，增加tickCount
			tickCount++;
		}
		lastSysTickVal = currentSysTickVal;

		return tickCount;
	}
}		   
