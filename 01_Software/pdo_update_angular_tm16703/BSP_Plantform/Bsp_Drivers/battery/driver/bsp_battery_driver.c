/**
 * @file bsp_battery_driver.c
 * @brief Battery Modbus byte stream parser and query driver.
 */

/********************************** Includes *********************************/
#include "bsp_battery_driver.h"
#include "uart_legacy_bridge.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system_cfg.h"

/********************************** Variables ********************************/
#ifndef DEBUG
static ModbusRecData rec_data = {0};
#else
ModbusRecData rec_data = {0};
#endif

static const uint8_t s_modbus_voltage_query_frame[] = {
    0xdd, 0x0d, 0x03, 0x03, 0x01, 0x00, 0x15, 0xf8, 0x77
};

static modbus_frame_cb_t frame_cb = NULL;

static uint32_t modbus_driver_now_tick(void)
{
    if (xPortIsInsideInterrupt() != pdFALSE)
    {
        return (uint32_t)xTaskGetTickCountFromISR();
    }

    return (uint32_t)xTaskGetTickCount();
}

/********************************** Functions ********************************/

void modbus_driver_init(void)
{
    memset(&rec_data, 0, sizeof(rec_data));
}

ModbusRecData* ModbusDriver_GetRecData(void)
{
    return &rec_data;
}

void modbus_driver_send_voltage_query(void)
{
    (void)uart_legacy_battery_write(s_modbus_voltage_query_frame,
                                    (uint16_t)sizeof(s_modbus_voltage_query_frame));
}

void ModbusDriver_UpdateTimestamp(uint32_t current_time)
{
    rec_data.last_recv_time = current_time;
    rec_data.comm_status = 0;
}

uint8_t ModbusDriver_CheckCommStatus(void)
{
    uint32_t current_time = xTaskGetTickCount();
    
    if((current_time - rec_data.last_recv_time) > pdMS_TO_TICKS(MODBUS_TIMEOUT_MS))
    {
        rec_data.comm_status = 1;
    }
    
    return rec_data.comm_status;
}

void modbus_driver_register_callback(modbus_frame_cb_t cb)
{
    frame_cb = cb;
}

void modbus_driver_input_byte(uint8_t byte)
{
    ModbusDriver_UpdateTimestamp(modbus_driver_now_tick());

    switch (rec_data.rec_idle_flag)
    {
        case MODBUS_IDLE:
            if (byte == MODBUS_HEADER_1)
            {
                rec_data.rec_index = 0;
                rec_data.rec_buf[rec_data.rec_index++] = byte;
                rec_data.rec_idle_flag = MODBUS_RECEIVEING;
            }
            break;

        case MODBUS_RECEIVEING:
            if (rec_data.rec_index < MODBUS_FRAME_LEN)
            {
                rec_data.rec_buf[rec_data.rec_index++] = byte;

                if (rec_data.rec_index == MODBUS_FRAME_LEN)
                {
                    rec_data.rec_idle_flag = MODBUS_IDLE;

                    uint8_t head = rec_data.rec_buf[1];
                    uint8_t tail = rec_data.rec_buf[rec_data.rec_index - 1];

                    if (head == MODBUS_HEADER_2 && tail == MODBUS_TAIL && frame_cb)
                    {
                        frame_cb(rec_data.rec_buf, MODBUS_FRAME_LEN);
                    }
                }
            }
            else
            {
                rec_data.rec_idle_flag = MODBUS_IDLE;
            }
            break;

        default:
            memset(rec_data.rec_buf, 0, sizeof(rec_data.rec_buf));
            break;
    }
}
