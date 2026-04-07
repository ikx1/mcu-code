#include "pwm_port.h"

#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} pwm_device_cfg_t;

static const pwm_device_cfg_t s_pwm_cfg[PWM_DEV_MAX] = {
    [PWM_DEV_WS2812] = {
        .htim = &htim3,
        .channel = TIM_CHANNEL_3,
    },
};

static const pwm_device_cfg_t *pwm_port_get_cfg(pwm_device_t dev)
{
    if (((uint32_t)dev >= (uint32_t)PWM_DEV_MAX) ||
        (s_pwm_cfg[(uint32_t)dev].htim == NULL))
    {
        return NULL;
    }

    return &s_pwm_cfg[(uint32_t)dev];
}

static bool pwm_port_start_dma_impl(const pwm_device_cfg_t *cfg,
                                    const uint16_t *buf,
                                    uint16_t len)
{
    if ((cfg == NULL) || (buf == NULL) || (len == 0u))
    {
        return false;
    }

    return (HAL_TIM_PWM_Start_DMA(cfg->htim,
                                  cfg->channel,
                                  (uint32_t *)(uintptr_t)buf,
                                  len) == HAL_OK);
}

static bool pwm_port_start_impl(const pwm_device_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    return (HAL_TIM_PWM_Start(cfg->htim, cfg->channel) == HAL_OK);
}

static void pwm_port_stop_dma_impl(const pwm_device_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, 0u);
    (void)HAL_TIM_PWM_Stop_DMA(cfg->htim, cfg->channel);
}

static void pwm_port_stop_impl(const pwm_device_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, 0u);
    (void)HAL_TIM_PWM_Stop(cfg->htim, cfg->channel);
}

static bool pwm_port_set_compare_impl(const pwm_device_cfg_t *cfg, uint32_t compare)
{
    uint32_t arr = 0u;

    if (cfg == NULL)
    {
        return false;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(cfg->htim);
    if (compare > arr)
    {
        compare = arr;
    }

    __HAL_TIM_SET_COMPARE(cfg->htim, cfg->channel, compare);
    return true;
}

static pwm_device_t pwm_port_find_dev_by_tim(TIM_HandleTypeDef *htim)
{
    uint32_t i;

    if (htim == NULL)
    {
        return PWM_DEV_MAX;
    }

    for (i = 0u; i < (uint32_t)PWM_DEV_MAX; ++i)
    {
        const pwm_device_cfg_t *cfg = &s_pwm_cfg[i];
        if ((cfg->htim != NULL) &&
            ((htim == cfg->htim) || (htim->Instance == cfg->htim->Instance)))
        {
            return (pwm_device_t)i;
        }
    }

    return PWM_DEV_MAX;
}

bool pwm_port_start_dma(pwm_device_t dev, const uint16_t *buf, uint16_t len)
{
    return pwm_port_start_dma_impl(pwm_port_get_cfg(dev), buf, len);
}

bool pwm_port_start(pwm_device_t dev)
{
    return pwm_port_start_impl(pwm_port_get_cfg(dev));
}

void pwm_port_stop(pwm_device_t dev)
{
    pwm_port_stop_impl(pwm_port_get_cfg(dev));
}

bool pwm_port_set_compare(pwm_device_t dev, uint32_t compare)
{
    return pwm_port_set_compare_impl(pwm_port_get_cfg(dev), compare);
}

bool pwm_port_set_duty_permille(pwm_device_t dev, uint16_t duty_permille)
{
    const pwm_device_cfg_t *cfg = pwm_port_get_cfg(dev);
    uint32_t arr = 0u;
    uint32_t compare = 0u;

    if ((cfg == NULL) || (duty_permille > 1000u))
    {
        return false;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(cfg->htim);
    compare = (uint32_t)(((uint64_t)arr * (uint64_t)duty_permille) / 1000u);
    return pwm_port_set_compare_impl(cfg, compare);
}

void pwm_port_stop_dma(pwm_device_t dev)
{
    pwm_port_stop_dma_impl(pwm_port_get_cfg(dev));
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    pwm_device_t dev = pwm_port_find_dev_by_tim(htim);

    if ((uint32_t)dev < (uint32_t)PWM_DEV_MAX)
    {
        pwm_port_stop_dma(dev);
    }
}
