/**
 * @file bsp_ibus_handler.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_ibus_handler.h"

#include "FreeRTOS.h"
#include "semphr.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
static IBUS_Handler ibus_handler;
//IBUS_Handler ibus_handler;


/********************************** Functions ********************************/
void ibus_handler_process(IBUS_Handler* handler, const uint16_t* raw_channels);

IBUS_Handler* get_ibus_data_p(void)
{
    return &ibus_handler;
}

void ibus_callback(const uint8_t* frame, uint8_t len) 
{
    uint16_t raw_channels[CHANNEL_USER];
    for (int i = 0; i < CHANNEL_USER; i++) 
    {
        raw_channels[i] = frame[2 + i * 2] | (frame[3 + i * 2] << 8);
    }

    ibus_handler_process(&ibus_handler, raw_channels);
    ibus_handler.last_update_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// 处理开关值
static int16_t process_switch(uint16_t val) 
{
    if (val > 1950 && val < 2050) return 2;   // 下
    if (val > 1450 && val < 1550) return 1;   // 中
    if (val > 950 && val < 1050) return 0;    // 上
    return -1; // 无效
}

// 处理开关值
static int16_t process_switch_2(uint16_t val) 
{
    if (val > 1950 && val < 2050) return 1;   // 下
    if (val > 950 && val < 1050) return 0;    // 上
    return -1; // 无效
}

void ibus_handler_init(IBUS_Handler* handler, uint32_t timeout_ms)
{
    memset(handler, 0, sizeof(IBUS_Handler));
    handler->timeout_ms = timeout_ms;
}

void ibus_handler_process(IBUS_Handler* handler, const uint16_t* raw_channels) 
{
    // 摇杆和旋钮处理（转换为±100范围）
    handler->channels[IBUS_CH_RX] = (raw_channels[0] - 1500) / 5;
    handler->channels[IBUS_CH_RY] = (raw_channels[1] - 1500) / 5;
    handler->channels[IBUS_CH_LY] = (raw_channels[2] - 1500) / 5;
    handler->channels[IBUS_CH_LX] = (raw_channels[3] - 1500) / 5;
    handler->channels[IBUS_CH_VRA] = (raw_channels[8] - 1500) / 5;
    
    // 开关处理
    handler->channels[IBUS_CH_SWD] = process_switch_2(raw_channels[4]);
    handler->channels[IBUS_CH_SWC] = process_switch(raw_channels[5]);
    handler->channels[IBUS_CH_SWB] = process_switch(raw_channels[6]);
    handler->channels[IBUS_CH_SWA] = process_switch_2(raw_channels[7]);
    
    // 连接状态
    handler->channels[IBUS_CH_CONN] = 1;
    handler->connected = true;
}

void ibus_handler_check_timeout(IBUS_Handler* handler, uint32_t now_ms) 
{
    if ((now_ms - handler->last_update_time_ms) > handler->timeout_ms) 
    {
        handler->connected = false;
        handler->channels[IBUS_CH_CONN] = 0;
    }
}
// 获取指定通道的值
int16_t ibus_handler_get_channel_value(IBUS_Handler* handler, ibus_channel_e channel)
{
    if (channel < CHANNEL_USER) 
    {
        return handler->channels[channel];
    }
    return -1;  // 如果传入了无效通道，返回无效值
}

void ibus_handler_receive_timeout(IBUS_Handler* handler)
{
    handler->channels[IBUS_CH_RX] = 0;
    handler->channels[IBUS_CH_RY] = 0;
    handler->channels[IBUS_CH_LY] = 0;
    handler->channels[IBUS_CH_LX] = 0;
    handler->channels[IBUS_CH_VRA] = 0;
    handler->channels[IBUS_CH_SWD] = 0;
    handler->channels[IBUS_CH_SWC] = 0;
    handler->channels[IBUS_CH_SWB] = 0;
    handler->channels[IBUS_CH_SWA] = 0;
}


