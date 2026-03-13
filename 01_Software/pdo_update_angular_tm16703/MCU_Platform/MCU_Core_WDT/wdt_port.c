#include "wdt_port.h"

#include "iwdg.h"

#include "stm32f10x_iwdg.h"

void wdt_port_init(void)
{
    IWDG_Init_StdPeriph();
}

void wdt_port_enable(void)
{
    IWDG_Enable();
}

void wdt_port_feed(void)
{
    IWDG_ReloadCounter();
}
