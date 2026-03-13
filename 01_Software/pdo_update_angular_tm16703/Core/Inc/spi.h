/**
 * @file spi.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __SPI_H__
#define __SPI_H__

/********************************** Includes *********************************/
#include "stm32f10x.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
void spi_spitx_config(uint8_t *buf, uint32_t buf_size);
void spi_init(void);

void ws2812_Send_Data(uint16_t size);

#endif
