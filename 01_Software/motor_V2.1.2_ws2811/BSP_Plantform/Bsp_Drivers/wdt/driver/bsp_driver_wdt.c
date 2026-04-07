/********************************** Includes *********************************/
#include "bsp_driver_wdt.h"
#include "wdt_port.h"

/********************************** Defines **********************************/

/********************************** Variables ********************************/

/********************************** Functions ********************************/
#if NRF_MODULE_ENABLED(GWP_WDT_MONITOR)

ret_code_t hal_wdt_feed(void)
{
    wdt_port_feed();
    return WDT_SUCCESS;
}

ret_code_t hal_wdt_enable(void)
{
    wdt_port_enable();
    return WDT_SUCCESS;
}

ret_code_t hal_wdt_init(void)
{
    wdt_port_init();
    return WDT_SUCCESS;
}

#endif /* NRF_MODULE_ENABLED(GWP_WDT_MONITOR) */

#if 0 /* legacy implementation (disabled) */

/**
 * @brief 喂狗操作，刷新独立看门狗计数器。
 * 
 * 此函数调用 HAL 库的 HAL_IWDG_Refresh 函数来刷新独立看门狗 (IWDG) 的计数器，
 * 防止看门狗超时触发系统复位。若操作成功，返回 WDT_SUCCESS 表示喂狗成功。
 * 
 * @retval ret_code_t 返回操作结果，成功时返回 WDT_SUCCESS。
 */

    // 调用 HAL 库函数刷新独立看门狗计数器
    wdt_port_feed();
    // 返回喂狗操作成功状态
    return WDT_SUCCESS; 
#endif /* legacy implementation kept for reference */
