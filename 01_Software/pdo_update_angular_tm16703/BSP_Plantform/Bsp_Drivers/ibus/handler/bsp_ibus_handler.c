#include "bsp_ibus_handler.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system_cfg.h"
#include "irq_guard.h"

#ifndef DEBUG
    static IBUS_Handler ibus_handler;
#else
    IBUS_Handler ibus_handler;
#endif

static void ibus_handler_init(IBUS_Handler *handler, uint32_t timeout_ms);
static void ibus_handler_process(IBUS_Handler *handler, const uint16_t *raw_channels);
static void ibus_handler_check_timeout(IBUS_Handler *handler, uint32_t now_ms);

/**
 * @brief 锁定IBUS处理程序中断
 * 
 * @return uint32_t 中断屏蔽标志
 */
static uint32_t ibus_handler_lock(void)
{
    return mcu_irq_guard_lock();
}

/**
 * @brief 解锁IBUS处理程序中断
 * 
 * @param primask 中断屏蔽标志
 */
static void ibus_handler_unlock(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

/**
 * @brief 获取当前系统时间戳
 * 
 * @return uint32_t 当前系统时间戳
 */
static uint32_t ibus_handler_now_tick(void)
{
    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        return (uint32_t)xTaskGetTickCountFromISR();
    }

    return (uint32_t)xTaskGetTickCount();
}

/**
 * @brief 处理开关值
 * 
 * @param val 开关值
 * @return int16_t 处理后的开关值
 */
static int16_t process_switch(uint16_t val)
{
    if (val > 1950u && val < 2050u) {
        return 2;
    }
    if (val > 1450u && val < 1550u) {
        return 1;
    }
    if (val > 950u && val < 1050u) {
        return 0;
    }
    return -1;
}

/**
 * @brief 处理开关值2
 * 
 * @param val 
 * @return int16_t 
 */
static int16_t process_switch_2(uint16_t val)
{
    if (val > 1950u && val < 2050u) {
        return 1;
    }
    if (val > 950u && val < 1050u) {
        return 0;
    }
    return -1;
}

/**
 * @brief 获取IBUS处理程序快照
 * 
 * @param out 输出IBUS处理程序结构体指针
 * @return uint8_t 0表示成功，1表示失败
 */
uint8_t ibus_handler_snapshot(IBUS_Handler *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return 1u;
    }

    primask = ibus_handler_lock();
    *out = ibus_handler;
    ibus_handler_unlock(primask);

    return 0u;
}

/**
 * @brief 初始化默认IBUS处理程序
 * 
 * @param timeout_ms 超时时间_ms
 */
void ibus_handler_init_default(uint32_t timeout_ms)
{
    ibus_handler_init(&ibus_handler, timeout_ms);
}

/**
 * @brief 检查默认IBUS处理程序超时
 * 
 * @param now_ms 当前时间戳_ms
 */
void ibus_handler_check_timeout_default(uint32_t now_ms)
{
    ibus_handler_check_timeout(&ibus_handler, now_ms);
}

/**
 * @brief IBUS回调函数
 * 
 * @param frame IBUS数据帧指针
 * @param len IBUS数据帧长度
 */
void ibus_callback(const uint8_t *frame, uint8_t len)
{
    uint16_t raw_channels[CHANNEL_USER];
    int i;

    if (frame == NULL || len < (2u + (uint8_t)(CHANNEL_USER * 2u))) {
        return;
    }

    for (i = 0; i < CHANNEL_USER; i++) {
        raw_channels[i] = (uint16_t)frame[2 + i * 2]
                        | (uint16_t)((uint16_t)frame[3 + i * 2] << 8);
    }

    ibus_handler_process(&ibus_handler, raw_channels);
}

/**
 * @brief 初始化IBUS处理程序
 * 
 * @param handler IBUS处理程序结构体指针
 * @param timeout_ms 超时时间_ms
 */
static void ibus_handler_init(IBUS_Handler *handler, uint32_t timeout_ms)
{
    uint32_t primask;

    if (handler == NULL) {
        return;
    }

    primask = ibus_handler_lock();
    memset(handler, 0, sizeof(*handler));
    handler->timeout_ms = timeout_ms;
    ibus_handler_unlock(primask);
}

/**
 * @brief 处理IBUS数据帧
 * 
 * @param handler IBUS处理程序结构体指针
 * @param raw_channels 原始通道值数组指针
 */
static void ibus_handler_process(IBUS_Handler *handler, const uint16_t *raw_channels)
{
    uint32_t primask;

    if (handler == NULL || raw_channels == NULL) {
        return;
    }

    primask = ibus_handler_lock();
    handler->channels[IBUS_CH_RX] = (int16_t)((int32_t)raw_channels[0] - 1500) / 5;
    handler->channels[IBUS_CH_RY] = (int16_t)((int32_t)raw_channels[1] - 1500) / 5;
    handler->channels[IBUS_CH_LY] = (int16_t)((int32_t)raw_channels[2] - 1500) / 5;
    handler->channels[IBUS_CH_LX] = (int16_t)((int32_t)raw_channels[3] - 1500) / 5;
    handler->channels[IBUS_CH_VRA] = (int16_t)((int32_t)raw_channels[8] - 1500) / 5;

    handler->channels[IBUS_CH_SWD] = process_switch_2(raw_channels[4]);
    handler->channels[IBUS_CH_SWC] = process_switch(raw_channels[5]);
    handler->channels[IBUS_CH_SWB] = process_switch(raw_channels[6]);
    handler->channels[IBUS_CH_SWA] = process_switch_2(raw_channels[7]);

    handler->channels[IBUS_CH_CONN] = 1;
    handler->connected = true;
    handler->last_update_time_ms = ibus_handler_now_tick() * portTICK_PERIOD_MS;
    ibus_handler_unlock(primask);
}

/**
 * @brief 检查IBUS处理程序超时
 * 
 * @param handler IBUS处理程序结构体指针
 * @param now_ms 当前时间戳_ms
 */
static void ibus_handler_check_timeout(IBUS_Handler *handler, uint32_t now_ms)
{
    uint32_t primask;

    if (handler == NULL) {
        return;
    }

    primask = ibus_handler_lock();
    if ((now_ms - handler->last_update_time_ms) > handler->timeout_ms) {
        handler->channels[IBUS_CH_RX] = 0;
        handler->channels[IBUS_CH_RY] = 0;
        handler->channels[IBUS_CH_LY] = 0;
        handler->channels[IBUS_CH_LX] = 0;
        handler->channels[IBUS_CH_VRA] = 0;
        handler->channels[IBUS_CH_SWD] = 0;
        handler->channels[IBUS_CH_SWC] = 0;
        handler->channels[IBUS_CH_SWB] = 0;
        handler->channels[IBUS_CH_SWA] = 0;
        handler->connected = false;
        handler->channels[IBUS_CH_CONN] = 0;
    }
    ibus_handler_unlock(primask);
}

/**
 * @brief 获取IBUS处理程序通道值
 * 
 * @param handler IBUS处理程序结构体指针
 * @param channel IBUS通道枚举值
 * @return int16_t 通道值，-1表示失败
 */
int16_t ibus_handler_get_channel_value(const IBUS_Handler *handler, ibus_channel_e channel)
{
    if (handler == NULL) {
        return -1;
    }

    if ((int)channel < CHANNEL_USER) {
        return handler->channels[channel];
    }
    return -1;
}
