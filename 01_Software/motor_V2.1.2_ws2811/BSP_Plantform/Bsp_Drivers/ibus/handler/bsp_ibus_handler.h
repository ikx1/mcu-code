#ifndef __BSP_IBUS_HANDLER_H__
#define __BSP_IBUS_HANDLER_H__

#include <stdbool.h>
#include <stdint.h>

#include "bsp_ibus_driver.h"

#define CHANNEL_USER 10

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
    int16_t channels[CHANNEL_USER];
    bool connected;
    uint32_t timeout_ms;
    uint32_t last_update_time_ms;
} IBUS_Handler;

uint8_t ibus_handler_snapshot(IBUS_Handler *out);
void ibus_handler_init_default(uint32_t timeout_ms);
void ibus_handler_check_timeout_default(uint32_t now_ms);

int16_t ibus_handler_get_channel_value(const IBUS_Handler *handler, ibus_channel_e channel);
void ibus_callback(const uint8_t *frame, uint8_t len);

#endif /* __BSP_IBUS_HANDLER_H__ */
