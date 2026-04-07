#include "wdt_port.h"

#include "iwdg.h"

void wdt_port_init(void)
{
    /* Keep empty: IWDG instance is initialized by CubeMX generated flow. */
}

void wdt_port_enable(void)
{
    __HAL_IWDG_START(&hiwdg);
}

void wdt_port_feed(void)
{
    (void)HAL_IWDG_Refresh(&hiwdg);
}
