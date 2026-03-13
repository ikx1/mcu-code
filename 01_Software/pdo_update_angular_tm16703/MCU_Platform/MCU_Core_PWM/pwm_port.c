#include "pwm_port.h"

#include <stddef.h>

#include "delay.h"
#include "tim.h"

#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"

#define PWM_WS2812_RESET_LATCH_US 200u

static uint8_t s_pwm_hw_ready = 0u;

static uint8_t pwm_port_dev_valid(pwm_device_t dev)
{
    return ((uint32_t)dev < (uint32_t)PWM_DEV_MAX) ? 1u : 0u;
}

static void pwm_port_ws2812_hw_init_once(void)
{
    if (s_pwm_hw_ready != 0u)
    {
        return;
    }

    WS2811_TIM3_CH2_PWM_INIT();
    s_pwm_hw_ready = 1u;
}

static uint32_t pwm_port_ws2812_arr_get(void)
{
    return (uint32_t)TIM3->ARR;
}

static void pwm_port_ws2812_set_compare(uint32_t compare)
{
    uint32_t arr = pwm_port_ws2812_arr_get();

    if (compare > arr)
    {
        compare = arr;
    }

    TIM_SetCompare2(TIM3, (uint16_t)compare);
}

static void pwm_port_ws2812_enter_idle(void)
{
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_GL3);
    TIM_Cmd(TIM3, DISABLE);
}

static void pwm_port_ws2812_dma_reload(const uint16_t *buf, uint16_t len)
{
    DMA_Cmd(DMA1_Channel3, DISABLE);
    DMA1_Channel3->CMAR = (uint32_t)buf;
    DMA_SetCurrDataCounter(DMA1_Channel3, len);
}

bool pwm_port_start_dma(pwm_device_t dev, const uint16_t *buf, uint16_t len)
{
    pwm_port_ws2812_hw_init_once();

    if ((pwm_port_dev_valid(dev) == 0u) || (buf == NULL) || (len == 0u))
    {
        return false;
    }

    pwm_port_ws2812_enter_idle();
    pwm_port_ws2812_dma_reload(buf, len);
    DMA_ClearFlag(DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_GL3);

    TIM_SetCounter(TIM3, 0u);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    DMA_Cmd(DMA1_Channel3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    while (DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET)
    {
    }

    while (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) == RESET)
    {
    }
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    pwm_port_ws2812_enter_idle();
    delay_us(PWM_WS2812_RESET_LATCH_US);

    return true;
}

bool pwm_port_start(pwm_device_t dev)
{
    pwm_port_ws2812_hw_init_once();

    if (pwm_port_dev_valid(dev) == 0u)
    {
        return false;
    }

    TIM_Cmd(TIM3, ENABLE);
    return true;
}

void pwm_port_stop(pwm_device_t dev)
{
    if (pwm_port_dev_valid(dev) == 0u)
    {
        return;
    }

    pwm_port_ws2812_hw_init_once();
    pwm_port_ws2812_enter_idle();
}

bool pwm_port_set_compare(pwm_device_t dev, uint32_t compare)
{
    pwm_port_ws2812_hw_init_once();

    if (pwm_port_dev_valid(dev) == 0u)
    {
        return false;
    }

    pwm_port_ws2812_set_compare(compare);
    return true;
}

bool pwm_port_set_duty_permille(pwm_device_t dev, uint16_t duty_permille)
{
    uint32_t arr;
    uint32_t compare;

    pwm_port_ws2812_hw_init_once();

    if ((pwm_port_dev_valid(dev) == 0u) || (duty_permille > 1000u))
    {
        return false;
    }

    arr = pwm_port_ws2812_arr_get();
    compare = (uint32_t)(((uint64_t)arr * (uint64_t)duty_permille) / 1000u);
    pwm_port_ws2812_set_compare(compare);

    return true;
}

void pwm_port_stop_dma(pwm_device_t dev)
{
    if (pwm_port_dev_valid(dev) == 0u)
    {
        return;
    }

    pwm_port_ws2812_hw_init_once();
    pwm_port_ws2812_enter_idle();
}
