#ifndef __WDT_TASK_H__
#define __WDT_TASK_H__

/********************************** Includes *********************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "bsp_driver_wdt.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/********************************** Defines **********************************/
#define WDT_MONITOR_WAIT_MS 50
#define WDT_INITED 1
#define WDT_NOT_INITED 0

#define WDT_MONITOR_TASK_MAX 10

#define WDT_TICK xTaskGetTickCount()

/********************************** Variables ********************************/
typedef enum
{
    WDT_THREAD_A,
    WDT_THREAD_MAX,
} wdt_thread_type_t;

/********************************** Functions ********************************/
ret_code_t wdt_monitor_init(void);

ret_code_t wdt_monitor_task_register(const char *name,
                                     uint32_t max_period,
                                     uint32_t *out_id);

ret_code_t wdt_monitor_task_enable(uint32_t id);

ret_code_t wdt_monitor_task_disable(uint32_t id);

ret_code_t wdt_monitor_task_period_set(uint32_t id, uint32_t max_period);

ret_code_t wdt_monitor_task_feed(uint32_t id);

ret_code_t wdt_monitor_notify(void);

void wdt_monitor_thread(void *arg);

#endif /* __WDT_TASK_H__ */
