#ifndef __BSP_DRIVER_WDT_H__
#define __BSP_DRIVER_WDT_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Defines **********************************/
#define NRF_MODULE_ENABLED(flag) (flag?1:0)

#define GWP_WDT_MONITOR 1

#define GWP_WDT_MONITOR_ENABLED 1

/********************************** Variables ********************************/
typedef enum
{
    WDT_SUCCESS,
    WDT_ERROR_INVALID_STATE,
    WDT_ERROR_NO_MEM,
    WDT_ERROR_BUSY,
    WDT_ERROR_RESOURCES,
    WDT_ERROR_INVALID_PARAM,
} ret_code_t;


/********************************** Functions ********************************/
ret_code_t hal_wdt_init(void);

ret_code_t hal_wdt_enable(void);

ret_code_t hal_wdt_feed(void);

#endif /* __BSP_DRIVER_WDT_H__ */
