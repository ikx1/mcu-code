/**
 * @file speed_filter.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __SPEED_FILTER_H__
#define __SPEED_FILTER_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Variables ********************************/
typedef struct {
    float output;        /* 上次输出 */
    float base_alpha;    /* 最低平滑系数 */
    float max_alpha;     /* 最高平滑系数，限制瞬时响应 */
    float gain;          /* 增益，输入变化越大，alpha 增大越快 */
    float noise_floor;   /* 噪声门限，小于该值视为 0 抑制抖动 */
} float_filter_t;

typedef struct {
    float value;       /* 当前滤波值 */
    float alpha;       /* 平滑系数 (0 < alpha < 1) */
    uint8_t has_init;  /* 是否初始化过 */
} ema_filter_t;


/********************************** Functions ********************************/
void float_filter_init(float_filter_t *filter,
                       float base_alpha,
                       float max_alpha,
                       float gain,
                       float noise_floor);
float float_filter_update(float_filter_t *filter, float input);

void ema_filter_init(ema_filter_t *filter, float alpha);
float ema_filter_update(ema_filter_t *filter, float input);


#endif /* __SPEED_FILTER_H__ */
