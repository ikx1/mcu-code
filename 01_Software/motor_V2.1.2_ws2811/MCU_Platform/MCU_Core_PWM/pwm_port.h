#ifndef PWM_PORT_H
#define PWM_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PWM_DEV_WS2812 = 0,
    PWM_DEV_MAX,
} pwm_device_t;

bool pwm_port_start(pwm_device_t dev);
void pwm_port_stop(pwm_device_t dev);
bool pwm_port_set_compare(pwm_device_t dev, uint32_t compare);
bool pwm_port_set_duty_permille(pwm_device_t dev, uint16_t duty_permille);

bool pwm_port_start_dma(pwm_device_t dev, const uint16_t *buf, uint16_t len);
void pwm_port_stop_dma(pwm_device_t dev);

#ifdef __cplusplus
}
#endif

#endif /* PWM_PORT_H */
