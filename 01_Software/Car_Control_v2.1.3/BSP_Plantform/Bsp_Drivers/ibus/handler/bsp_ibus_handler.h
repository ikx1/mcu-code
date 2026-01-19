/**
 * @file bsp_ibus_handler.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __BSP_IBUS_HANDLER_H__
#define __BSP_IBUS_HANDLER_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bsp_ibus_driver.h"

/********************************** Defines **********************************/
#define CHANNEL_USER			10

/********************************** Variables ********************************/
typedef enum 
{
	IBUS_CH_RX = 0,
    IBUS_CH_RY,
    IBUS_CH_LY,
    IBUS_CH_LX,
    IBUS_CH_SWD,
    IBUS_CH_SWC,
    IBUS_CH_SWB,
    IBUS_CH_SWA,
    IBUS_CH_VRA,
    IBUS_CH_CONN,   

} ibus_channel_e;

typedef struct 
{
    int16_t channels[CHANNEL_USER];   // 摇杆为 -100~100，开关为 0~2
    bool connected;                  // 当前是否连接
    uint32_t timeout_ms;             // 失联超时阈值
    uint32_t last_update_time_ms;    // 上一次更新的时间戳（ms）
} IBUS_Handler;

/********************************** Functions ********************************/
IBUS_Handler* get_ibus_data_p(void);


/**
 * @brief 初始化 ibus handler 结构体
 */
void ibus_handler_init(IBUS_Handler* handler, uint32_t timeout_ms);

/**
 * @brief 处理一帧 iBus 原始数据（由 driver 层回调调用）
 */
void ibus_handler_process(IBUS_Handler* handler, const uint16_t* raw_channels);

/**
 * @brief 周期调用，判断是否失联
 */
void ibus_handler_check_timeout(IBUS_Handler* handler, uint32_t now_ms);
int16_t ibus_handler_get_channel_value(IBUS_Handler* handler, ibus_channel_e channel);
void ibus_handler_receive_timeout(IBUS_Handler* handler);
void ibus_callback(const uint8_t* frame, uint8_t len);

#endif /* __BSP_IBUS_HANDLER_H__ */
