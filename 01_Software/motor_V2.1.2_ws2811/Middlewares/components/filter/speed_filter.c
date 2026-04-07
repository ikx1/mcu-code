/**
 * @file speed_filter.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "speed_filter.h"
#include <math.h>

/********************************** Functions ********************************/
void float_filter_init(float_filter_t *filter,
                       float base_alpha,
                       float max_alpha,
                       float gain,
                       float noise_floor)
{
    if (!filter) return;

    filter->output      = 0.0f;
    filter->base_alpha  = base_alpha;
    filter->max_alpha   = (max_alpha > 1.0f) ? 1.0f : max_alpha;
    filter->gain        = gain;
    filter->noise_floor = (noise_floor < 0.0f) ? 0.0f : noise_floor;
}

float float_filter_update(float_filter_t *filter, float input)
{
    if (!filter) return input;

    /* 噪声门限：输入与当前输出都很小则钳为 0，消除低速抖动 */
    if (fabsf(input) < filter->noise_floor &&
        fabsf(filter->output) < filter->noise_floor) {
        input = 0.0f;
    }

    float diff = fabsf(input - filter->output);
    float dynamic_alpha = filter->base_alpha + filter->gain * diff;

    /* 限幅，确保不过快也不过慢 */
    if (dynamic_alpha > filter->max_alpha) {
        dynamic_alpha = filter->max_alpha;
    }
    if (dynamic_alpha < filter->base_alpha) {
        dynamic_alpha = filter->base_alpha;
    }

    filter->output = dynamic_alpha * input + (1.0f - dynamic_alpha) * filter->output;
    return filter->output;
}


void ema_filter_init(ema_filter_t *filter, float alpha)
{
    if (!filter) return;
    filter->value    = 0.0f;
    filter->alpha    = alpha;
    filter->has_init = 0;
}

float ema_filter_update(ema_filter_t *filter, float input)
{
    if (!filter) return input;

    if (!filter->has_init) {
        filter->value    = input;
        filter->has_init = 1;
    } else {
        filter->value = filter->alpha * input + (1.0f - filter->alpha) * filter->value;
    }

    return filter->value;
}
