/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016-2018, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2016-04-23
 */

#include <sfud.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "spi.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"

#include "perf_counter.h"

#define INITED 1
#define UNINITED 0

static char log_buf[256];

typedef struct
{
    uint8_t inited;
    SemaphoreHandle_t *dma_semphr;
    SemaphoreHandle_t *lock_mutex;
    SPI_HandleTypeDef *hspi;

    GPIO_TypeDef *cs_gpio_port;
    uint16_t cs_gpio_pin;
}sufd_user_data_t;

SemaphoreHandle_t SPI1_DMA_Semaph;
SemaphoreHandle_t SPI1_lock_mutex;

sufd_user_data_t spi1_user_data = {
    .inited = UNINITED,
    .dma_semphr = &SPI1_DMA_Semaph,
    .lock_mutex = &SPI1_lock_mutex,
    .hspi = &hspi1,
    .cs_gpio_port = SPI1_CS_GPIO_Port,
    .cs_gpio_pin = SPI1_CS_Pin
};

/**
 * @brief spi dma transfer complete callback
 *        This function is executed when the SPI DMA transfer is complete.
 *        It releases the DMA semaphore to signal that the transfer is complete.
 *        If a higher priority task is waiting for the semaphore, it will be woken up.
 * 
 * @param hspi 
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 初始化yield变量为NULL，用于指示是否需要在中断服务例程中进行上下文切换
    BaseType_t yield = NULL;
    
    // 检查传入的SPI句柄是否为SPI1的句柄
    if(hspi == &hspi1)
    {
        // 释放SPI1_DMA_Semaph信号量，通知其他任务SPI传输已经完成
        // 这里使用的是中断服务例程安全的版本
        xSemaphoreGiveFromISR(SPI1_DMA_Semaph, &yield);

        // 根据yield的值决定是否需要在中断服务例程中进行上下文切换
        portYIELD_FROM_ISR(yield);
    }
}

/**
 * @brief spi dma receive complete callback
 *        This function is executed when the SPI DMA receive is complete.
 *        It releases the DMA semaphore to signal that the transfer is complete.
 *        If a higher priority task is waiting for the semaphore, it will be woken up.
 * 
 * @param hspi 
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // 初始化yield变量为NULL，用于指示是否需要在中断服务例程中进行上下文切换
    BaseType_t yield = NULL;
    
    // 检查传入的SPI句柄是否为SPI1的句柄
    if(hspi == &hspi1)
    {
        // 释放SPI1_DMA_Semaph信号量，通知其他任务SPI传输已经完成
        // 这里使用的是中断服务例程安全的版本
        xSemaphoreGiveFromISR(SPI1_DMA_Semaph, &yield);

        // 根据yield的值决定是否需要在中断服务例程中进行上下文切换
        portYIELD_FROM_ISR(yield);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    BaseType_t yield = NULL;
    
    if(hspi == &hspi1)
    {
        xSemaphoreGiveFromISR(SPI1_DMA_Semaph,&yield);
        portYIELD_FROM_ISR(yield);
    }
}

void sfud_log_debug(const char *file, const long line, const char *format, ...);

/**
 * @brief SPI write data then read data
 * 
 * @param spi user data
 * @param write_buf write data buffer
 * @param write_size write data size
 * @param read_buf read data buffer
 * @param read_size read data size
 * @return sfud_err 
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,
        size_t read_size) {
    sfud_err result = SFUD_SUCCESS;
//    uint8_t send_data, read_data;

    /*  获取用户指针，用以访问SPI设备和GPIO引脚 */
    sufd_user_data_t *user_data = (sufd_user_data_t *)spi->user_data;
    /* 使能SPI设备 */
    HAL_GPIO_WritePin(user_data->cs_gpio_port, user_data->cs_gpio_pin, GPIO_PIN_RESET);

    /* 判断传入是否为空 */
    if(write_size)
    {
        HAL_SPI_Transmit_DMA(user_data->hspi, (uint8_t *)write_buf, write_size);
        /* 等待DMA传输完成 */
        xSemaphoreTake(*(user_data->dma_semphr), portMAX_DELAY);
    }

    /* 判断传入是否为空 */
    if(read_size)
    {
        HAL_SPI_Receive_DMA(user_data->hspi, (uint8_t *)read_buf, read_size);
        /* 等待DMA传输完成 */
        xSemaphoreTake(*(user_data->dma_semphr), portMAX_DELAY);
    }

    /* 结束通信 */
    HAL_GPIO_WritePin(user_data->cs_gpio_port, user_data->cs_gpio_pin, GPIO_PIN_SET);

    return result;
}

/**
 * @brief 保护临界区，防止多个任务同时操作SPI设备
 * 
 * @param spi 
 */
static void spi_lock(const sfud_spi *spi) {
    sufd_user_data_t *user_data = (sufd_user_data_t *)spi->user_data;
    xSemaphoreTake(*(user_data->lock_mutex), portMAX_DELAY);
}

/**
 * @brief 释放临界区，允许其他任务操作SPI设备

 * 
 * @param spi 
 */
static void spi_unlock(const sfud_spi *spi) {
    sufd_user_data_t *user_data = (sufd_user_data_t *)spi->user_data;
    xSemaphoreGive(user_data->lock_mutex);
}

static void retry_delay_100us(void)
{
    vTaskDelay(10);
}

#ifdef SFUD_USING_QSPI
/**
 * read flash data by QSPI
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format,
        uint8_t *read_buf, size_t read_size) {
    sfud_err result = SFUD_SUCCESS;

    /**
     * add your qspi read flash data code
     */

    return result;
}
#endif /* SFUD_USING_QSPI */

sfud_err sfud_spi_port_init(sfud_flash *flash) {
    sfud_err result = SFUD_SUCCESS;

    /**
     * add your port spi bus and device object initialize code like this:
     * 1. rcc initialize
     * 2. gpio initialize
     * 3. spi device initialize
     * 4. flash->spi and flash->retry item initialize
     *    flash->spi.wr = spi_write_read; //Required
     *    flash->spi.qspi_read = qspi_read; //Required when QSPI mode enable
     *    flash->spi.lock = spi_lock;
     *    flash->spi.unlock = spi_unlock;
     *    flash->spi.user_data = &spix;
     *    flash->retry.delay = null;
     *    flash->retry.times = 10000; //Required
     */
    
    /* 判断传入参数是否有效 */
    if(NULL == flash || NULL == flash->spi.name)
    {
        return SFUD_ERR_NOT_FOUND;
    }

    /* 判断传入设备名称是否为SPI1*/
    if(strcmp(flash->spi.name, "SPI1") == 0)
    {
        if(spi1_user_data.inited == UNINITED)
        {
            /* DMA使用二值信号量 */
            SPI1_DMA_Semaph = xSemaphoreCreateBinary();

            /* SPI使用互斥锁 */
            SPI1_lock_mutex = xSemaphoreCreateMutex();

            if(SPI1_DMA_Semaph == NULL || SPI1_lock_mutex == NULL)
            {
                if(SPI1_DMA_Semaph != NULL)
                {
                    vSemaphoreDelete(SPI1_DMA_Semaph);
                }
                if(SPI1_lock_mutex != NULL)
                {
                    vSemaphoreDelete(SPI1_lock_mutex);
                }

                return SFUD_ERR_NOT_FOUND;
            }

            spi1_user_data.dma_semphr = &SPI1_DMA_Semaph;
            spi1_user_data.lock_mutex = &SPI1_lock_mutex;
            spi1_user_data.inited = INITED;
        }

        /*这个地方需要注意，这里举例子只使用一个Flash，没有对多Flash多SPI支持*/
        /*如果使用了多Flash，需要考虑CS引脚的幅值*/
        spi1_user_data.cs_gpio_port = SPI1_CS_GPIO_Port;
        spi1_user_data.cs_gpio_pin = SPI1_CS_Pin;

        flash->spi.wr = spi_write_read;
        flash->spi.lock = spi_lock;
        flash->spi.unlock = spi_unlock;
        flash->spi.user_data = &spi1_user_data;
        flash->retry.delay = retry_delay_100us;
        flash->retry.times = 10000;
    }

    return result;
}

/**
 * This function is print debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    va_end(args);
}

/**
 * This function is print routine info.
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...) {
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD]");
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    va_end(args);
}
