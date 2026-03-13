#include "io_task.h"

#include "FreeRTOS.h"
#include "task.h"

void ScramTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    while (1)
    {
        input_driver_update_10ms();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}
