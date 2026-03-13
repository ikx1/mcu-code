#include "User_Init.h"

#include <stddef.h>
#include <stdbool.h>

#include "motor_task.h"
#include "uart_task.h"
#include "ibus_task.h"
#include "control_task.h"
#include "io_task.h"
#include "battery_task.h"
#include "ws2812_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "can_queue.h"
#include "bsp_motor_handler.h"
#include "bsp_ibus_driver.h"
#include "bsp_ibus_handler.h"
#include "bsp_battery_driver.h"
#include "bsp_battery_handler.h"
#include "bsp_ultrasonic_driver.h"
#include "bsp_display_driver.h"
#include "bsp_handler_display.h"
#include "uart_legacy_bridge.h"

static void user_init_halt(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

static bool user_task_create_checked(TaskFunction_t task,
                                     const char *name,
                                     uint16_t stack_words,
                                     UBaseType_t prio)
{
    return (xTaskCreate(task, name, stack_words, NULL, prio, NULL) == pdPASS);
}

static bool user_uart_service_init(void)
{
    return uart_task_start();
}

static void user_legacy_service_init(void)
{
    /* UART1 carries the new host protocol, while these legacy byte-stream devices
     * still depend on their original bridges and callbacks. Keep both paths alive. */
    ibus_driver_init();
    ibus_driver_register_callback(ibus_callback);
    uart_legacy_ibus_set_rx_handler(ibus_driver_input_byte);
    uart_legacy_ibus_init();

    modbus_driver_init();
    modbus_driver_register_callback(modbus_callback);
    uart_legacy_battery_set_rx_handler(modbus_driver_input_byte);
    BatteryHandler_Init();
    uart_legacy_battery_init();

//    ultrasonic_driver_init();
//    uart_legacy_ultrasonic_init();

    display_driver_register_callback(display_callback);
    uart_legacy_display_set_rx_handler(display_driver_input_byte);
    uart_legacy_display_init();
}

static void user_legacy_rx_service_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        /* Legacy peripherals are byte-stream parsers without dedicated RX tasks,
         * so poll them at 1ms to keep latency bounded and buffers drained. */
        uart_legacy_battery_poll_rx();
        uart_legacy_display_poll_rx();
        uart_legacy_ibus_poll_rx();

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1u));
    }
}

static void user_app_task_entry(void *pvParameters)
{
    (void)pvParameters;

    /* Create tasks from supervision/control to execution/peripheral service so
     * downstream tasks only start after their control-side dependencies exist. */
    if (!user_task_create_checked(ScramTask, "scram", 128u, 24u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(ContronlTask, "contronl", 128u * 5u, 25u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(Motor_Task, "motor", 128u * 4u, 24u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(Can_Analy_Task, "can", 128u * 4u, 24u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(Robot_Speed_Task, "filter_speed", 128u * 2u, 22u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(user_legacy_rx_service_task, "legacy_rx", 128u * 2u, 24u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(ibusTask, "ibus", 128u * 2u, 24u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(ModbusTask, "modbus", 128u * 4u, 22u))
    {
        user_init_halt();
    }
    if (!user_task_create_checked(WS2812_Task, "ws2812", 128u * 4u, 21u))
    {
        user_init_halt();
    }

    vTaskDelete(NULL);
}

void UserAppTask_Init(void)
{
    /* Bring up shared queues/motor registry first, then the new UART1 stack,
     * then spawn application tasks that consume those services. */
    if (can_queue_init() != 0u)
    {
        user_init_halt();
    }
    motor_register();

    if (!user_uart_service_init())
    {
        user_init_halt();
    }
    user_legacy_service_init();

    if (!user_task_create_checked(user_app_task_entry, "userTask", 128u * 4u, 26u))
    {
        user_init_halt();
    }

    /* The migrated project still uses raw FreeRTOS task creation, so the
     * scheduler must be started explicitly just like the stable F103 demo. */
    vTaskStartScheduler();

    /* Only reached if the scheduler could not start (for example, no heap left
     * for the idle/timer tasks). */
    user_init_halt();
}
