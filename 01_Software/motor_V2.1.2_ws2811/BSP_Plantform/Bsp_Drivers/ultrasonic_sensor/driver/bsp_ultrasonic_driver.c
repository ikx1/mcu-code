/**
 * @file bsp_ultrasonic_driver.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-07-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "bsp_ultrasonic_driver.h"
#include "uart_legacy_bridge.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static UltrasonicRecData s_ultrasonic_data;

static const uint8_t s_ultra_tx_buf[ULTRASONIC_TX_FRAME_LEN] =
{
    0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0x85, 0xF6
};

static uint16_t ultrasonic_driver_crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i = 0u;
    uint8_t bit = 0u;

    if (data == NULL) {
        return 0u;
    }

    for (i = 0u; i < len; i++) {
        crc ^= data[i];

        for (bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            } else {
                crc >>= 1u;
            }
        }
    }

    return crc;
}

static bsp_ultrasonic_status_t ultrasonic_driver_parse_frame(const uint8_t *rx_buf,
                                                             uint16_t *distance_mm)
{
    uint16_t recv_crc = 0u;
    uint16_t calc_crc = 0u;

    if (rx_buf == NULL || distance_mm == NULL) {
        return ULTRASONIC_INVALID_PARAM;
    }

    if (rx_buf[0] != ULTRASONIC_MODBUS_ADDR
        || rx_buf[1] != ULTRASONIC_MODBUS_FUNC
        || rx_buf[2] != ULTRASONIC_MODBUS_BYTE_COUNT) {
        return ULTRASONIC_FRAME_ERROR;
    }

    calc_crc = ultrasonic_driver_crc16_modbus(rx_buf, ULTRASONIC_RX_FRAME_LEN - 2u);
    recv_crc = (uint16_t)rx_buf[ULTRASONIC_RX_FRAME_LEN - 2u]
             | (uint16_t)((uint16_t)rx_buf[ULTRASONIC_RX_FRAME_LEN - 1u] << 8u);

    if (calc_crc != recv_crc) {
        return ULTRASONIC_CRC_ERROR;
    }

    *distance_mm = (uint16_t)((uint16_t)rx_buf[3] << 8u) | rx_buf[4];
    return ULTRASONIC_OK;
}

void ultrasonic_driver_init(void)
{
    memset(&s_ultrasonic_data, 0, sizeof(s_ultrasonic_data));
    s_ultrasonic_data.comm_status = 1u;
}

const UltrasonicRecData* ultrasonic_driver_get_data(void)
{
    return &s_ultrasonic_data;
}

uint8_t ultrasonic_driver_check_comm_status(void)
{
    uint32_t current_time = xTaskGetTickCount();

    if ((current_time - s_ultrasonic_data.last_recv_time) > pdMS_TO_TICKS(ULTRASONIC_COMM_TIMEOUT_MS)) {
        s_ultrasonic_data.comm_status = 1u;
    }

    return s_ultrasonic_data.comm_status;
}

bsp_ultrasonic_status_t ultrasonic_driver_read_distance(uint16_t *distance_mm)
{
    uint8_t rx_buf[ULTRASONIC_RX_FRAME_LEN] = {0};
    uint16_t parsed_distance = 0u;
    bsp_ultrasonic_status_t status = ULTRASONIC_ERROR;

    if (distance_mm == NULL) {
        return ULTRASONIC_INVALID_PARAM;
    }

    if (uart_legacy_ultrasonic_write(s_ultra_tx_buf, ULTRASONIC_TX_FRAME_LEN) != ULTRASONIC_TX_FRAME_LEN) {
        s_ultrasonic_data.comm_status = 1u;
        return ULTRASONIC_ERROR;
    }

    if (uart_legacy_ultrasonic_read(rx_buf, ULTRASONIC_RX_FRAME_LEN, ULTRASONIC_POLL_TIMEOUT_MS)
        != ULTRASONIC_RX_FRAME_LEN) {
        s_ultrasonic_data.comm_status = 1u;
        return ULTRASONIC_TIMEOUT;
    }

    status = ultrasonic_driver_parse_frame(rx_buf, &parsed_distance);
    if (status != ULTRASONIC_OK) {
        s_ultrasonic_data.comm_status = 1u;
        return status;
    }

    *distance_mm = parsed_distance;
    s_ultrasonic_data.distance_mm = parsed_distance;
    s_ultrasonic_data.last_recv_time = xTaskGetTickCount();
    s_ultrasonic_data.comm_status = 0u;

    return ULTRASONIC_OK;
}
