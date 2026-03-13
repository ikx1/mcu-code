#include "ibus_task.h"

#include "bsp_ibus_handler.h"

#include "FreeRTOS.h"
#include "task.h"

void ibusTask(void *pvParameters)
{
    TickType_t last_wake_time;

    (void)pvParameters;

    last_wake_time = xTaskGetTickCount();
    ibus_handler_init_default(100u);

    for (;;)
    {
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        ibus_handler_check_timeout_default(now_ms);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
