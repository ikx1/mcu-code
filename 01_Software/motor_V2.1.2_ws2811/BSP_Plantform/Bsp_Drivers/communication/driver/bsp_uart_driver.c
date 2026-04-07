#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_uart_driver.h"
#include "fifo.h"
#include "uart_port.h"

typedef struct
{
    volatile uint8_t tx_dma_busy;//发送DMA是否繁忙

    fifo_t tx_fifo;//发送FIFO
    fifo_t rx_fifo;//接收FIFO

    uint8_t *dmarx_buf;//DMA接收缓冲区
    uint16_t dmarx_buf_size;//DMA接收缓冲区大小

    uint8_t *dmatx_buf;//DMA发送缓冲区
    uint16_t dmatx_buf_size;//DMA发送缓冲区大小
    uint16_t tx_retry_len;//发送重试长度

    volatile uint16_t last_dmarx_size;//DMA接收缓冲区最后接收字节数
    volatile uint8_t inited;//是否初始化

    uart_diag_t diag;//诊断信息
} uart_device_t;

/* Per-port runtime context */
static uart_device_t s_uart_dev[UART_DEV_MAX] = {0};

/* Static buffers */
static uint8_t s_uart_tx_buf[UART_DEV_MAX][UART_TX_BUF_SIZE];
static uint8_t s_uart_rx_buf[UART_DEV_MAX][UART_RX_BUF_SIZE];
static uint8_t s_uart_dmarx_buf[UART_DEV_MAX][UART_DMA_RX_BUF_SIZE];
static uint8_t s_uart_dmatx_buf[UART_DEV_MAX][UART_DMA_TX_BUF_SIZE];

/**
 * @brief 检查UART ID是否有效
 * 
 * @param uart_id UART设备ID
 * @return true UART ID有效
 * @return false UART ID无效
 */
static inline bool uart_id_valid(uint8_t uart_id)
{
    return (uart_id < (uint8_t)UART_DEV_MAX);
}

/**
 * @brief 累积接收丢弃的字节数
 * 
 * @param dev UART设备指针
 * @param drop 要累积的丢弃字节数
 */
static void uart_acc_rx_drop(uart_device_t *dev, uint16_t drop)
{
    if (drop == 0u) {
        return;
    }
    dev->diag.rx_drop_bytes += drop;
    dev->diag.rx_overflow_cnt++;
}

/**
 * @brief 将数据写入UART接收FIFO
 * 
 * @param dev UART设备指针
 * @param src 要写入的数据指针
 * @param len 要写入的数据长度
 */
static void uart_rx_fifo_write(uart_device_t *dev, const uint8_t *src, uint16_t len)
{
    if (len == 0u) {
        return;
    }

    uint16_t wr = (uint16_t)fifo_write(&dev->rx_fifo, src, len);
    dev->diag.rx_queued_bytes += wr;
    if (wr < len) {
        uart_acc_rx_drop(dev, (uint16_t)(len - wr));
    }
}

/* recv_total: total received bytes in DMA ring [0..dmarx_buf_size]. */
/**
 * @brief 提交DMA接收字节到UART接收FIFO
 * 
 * @param dev UART设备指针
 * @param recv_total 已接收的总字节数
 */
static void uart_commit_dmarx_bytes(uart_device_t *dev, uint16_t recv_total)
{
    uint16_t cap;
    uint16_t last;
    uint16_t recv_size;

    if (dev == NULL || dev->dmarx_buf == NULL || dev->dmarx_buf_size == 0u) {
        return;
    }

    cap = dev->dmarx_buf_size;
    if (recv_total > cap) {
        recv_total = cap;
        dev->diag.rx_bad_isr_state_cnt++;
    }

    last = dev->last_dmarx_size;
    if (last >= cap) {
        last = 0u;
        dev->diag.rx_bad_isr_state_cnt++;
    }

    if (recv_total >= last) {
        recv_size = (uint16_t)(recv_total - last);
    } else {
        recv_size = (uint16_t)(cap - last + recv_total);
    }

    if (recv_size == 0u) {
        dev->last_dmarx_size = (recv_total == cap) ? 0u : recv_total;
        return;
    }

    dev->diag.rx_isr_bytes += recv_size;

    if ((uint32_t)last + recv_size <= cap) {
        uart_rx_fifo_write(dev, &dev->dmarx_buf[last], recv_size);
    } else {
        uint16_t first = (uint16_t)(cap - last);
        uint16_t second = (uint16_t)(recv_size - first);
        uart_rx_fifo_write(dev, &dev->dmarx_buf[last], first);
        uart_rx_fifo_write(dev, &dev->dmarx_buf[0], second);
    }

    dev->last_dmarx_size = (recv_total == cap) ? 0u : recv_total;
}

/**
 * @brief 获取DMA接收缓冲区中已接收的总字节数
 * 
 * @param uart_id UART设备ID
 * @param dev UART设备指针
 * @return uint16_t 已接收的总字节数
 */
static uint16_t uart_get_recv_total_from_dma(uint8_t uart_id, uart_device_t *dev)
{
    uint16_t remain;
    uint16_t total;

    if (dev == NULL || dev->dmarx_buf_size == 0u) {
        return 0u;
    }

    remain = uart_port_dma_rx_get_remain(uart_id);
    if (remain > dev->dmarx_buf_size) {
        remain = dev->dmarx_buf_size;
        dev->diag.rx_bad_isr_state_cnt++;
    }

    total = (uint16_t)(dev->dmarx_buf_size - remain);
    return total;
}

/**
 * @brief 初始化UART设备
 * 
 * @param uart_id UART设备ID
 */
void uart_device_init(uint8_t uart_id)
{
    uart_device_t *dev;

    if (!uart_id_valid(uart_id)) {
        return;
    }

    dev = &s_uart_dev[uart_id];
    if (dev->inited) {
        return;
    }

    fifo_register(&dev->tx_fifo, &s_uart_tx_buf[uart_id][0], UART_TX_BUF_SIZE, NULL, NULL);
    fifo_register(&dev->rx_fifo, &s_uart_rx_buf[uart_id][0], UART_RX_BUF_SIZE, NULL, NULL);

    dev->dmarx_buf = &s_uart_dmarx_buf[uart_id][0];
    dev->dmarx_buf_size = (uint16_t)UART_DMA_RX_BUF_SIZE;
    dev->dmatx_buf = &s_uart_dmatx_buf[uart_id][0];
    dev->dmatx_buf_size = (uint16_t)UART_DMA_TX_BUF_SIZE;

    dev->tx_dma_busy = 0u;
    dev->tx_retry_len = 0u;
    dev->last_dmarx_size = 0u;
    memset(&dev->diag, 0, sizeof(dev->diag));

    if (!uart_port_dma_rx_start(uart_id, dev->dmarx_buf, dev->dmarx_buf_size)) {
        dev->diag.rx_bad_isr_state_cnt++;
        return;
    }

    dev->inited = 1u;
}

/**
 * @brief 检查UART设备是否已初始化
 * 
 * @param uart_id UART设备ID
 * @return true UART设备已初始化
 * @return false UART设备未初始化
 */
bool uart_device_is_ready(uint8_t uart_id)
{
    return uart_id_valid(uart_id) && (s_uart_dev[uart_id].inited != 0u);
}
uint16_t uart_write(uint8_t uart_id, const uint8_t *buf, uint16_t size)
{
    uart_device_t *dev;
    uint16_t wr;

    if (!uart_id_valid(uart_id) || buf == NULL || size == 0u) {
        return 0u;
    }

    dev = &s_uart_dev[uart_id];
    if (!dev->inited) {
        return 0u;
    }

    dev->diag.tx_req_bytes += size;
    wr = (uint16_t)fifo_write(&dev->tx_fifo, buf, size);
    dev->diag.tx_queued_bytes += wr;
    if (wr < size) {
        dev->diag.tx_drop_bytes += (uint16_t)(size - wr);
    }
    return wr;
}

/**
 * @brief 从UART接收FIFO读取数据
 * 
 * @param uart_id UART设备ID
 * @param buf 接收数据缓冲区指针
 * @param size 要读取的数据长度
 * @return uint16_t 实际读取的数据长度
 */
uint16_t uart_read(uint8_t uart_id, uint8_t *buf, uint16_t size)
{
    if (!uart_id_valid(uart_id) || buf == NULL || size == 0u) {
        return 0u;
    }

    if (!s_uart_dev[uart_id].inited) {
        return 0u;
    }

    return (uint16_t)fifo_read(&s_uart_dev[uart_id].rx_fifo, buf, size);
}

/**
 * @brief 轮询UART DMA发送
 * 
 * @param uart_id UART设备ID
 */
void uart_poll_dma_tx(uint8_t uart_id)
{
    uart_device_t *dev;
    uint16_t size;

    if (!uart_id_valid(uart_id)) {
        return;
    }

    dev = &s_uart_dev[uart_id];
    if (!dev->inited) {
        return;
    }

    if (dev->tx_dma_busy) {
        dev->diag.tx_busy_skip_cnt++;
        return;
    }

    if (dev->tx_retry_len > 0u) {
        dev->tx_dma_busy = 1u;
        if (!uart_port_dma_tx_start(uart_id, dev->dmatx_buf, dev->tx_retry_len)) {
            dev->tx_dma_busy = 0u;
            dev->diag.tx_dma_start_fail_cnt++;
            return;
        }
        dev->tx_retry_len = 0u;
        return;
    }

    size = (uint16_t)fifo_read(&dev->tx_fifo, dev->dmatx_buf, dev->dmatx_buf_size);
    if (size == 0u) {
        return;
    }

    dev->tx_dma_busy = 1u;
    if (!uart_port_dma_tx_start(uart_id, dev->dmatx_buf, size)) {
        dev->tx_dma_busy = 0u;
        dev->tx_retry_len = size;
        dev->diag.tx_dma_start_fail_cnt++;
    }
}

/**
 * @brief UART DMA发送完成中断处理函数
 * 
 * @param uart_id UART设备ID
 */
void uart_dmatx_done_isr(uint8_t uart_id)
{
    if (!uart_id_valid(uart_id)) {
        return;
    }

    s_uart_dev[uart_id].tx_dma_busy = 0u;
    uart_port_dma_tx_stop(uart_id);
}

/**
 * @brief UART DMA接收完成中断处理函数
 * 
 * @param uart_id UART设备ID
 */
void uart_dmarx_done_isr(uint8_t uart_id)
{
    uart_device_t *dev;
    uint16_t recv_total;

    if (!uart_id_valid(uart_id)) {
        return;
    }

    dev = &s_uart_dev[uart_id];
    if (!dev->inited) {
        return;
    }

    recv_total = uart_get_recv_total_from_dma(uart_id, dev);
    uart_commit_dmarx_bytes(dev, recv_total);
}

/**
 * @brief UART DMA接收半完成中断处理函数
 * 
 * @param uart_id UART设备ID
 */
void uart_dmarx_half_done_isr(uint8_t uart_id)
{
    uart_device_t *dev;
    uint16_t recv_total;

    if (!uart_id_valid(uart_id)) {
        return;
    }

    dev = &s_uart_dev[uart_id];
    if (!dev->inited) {
        return;
    }

    recv_total = uart_get_recv_total_from_dma(uart_id, dev);
    uart_commit_dmarx_bytes(dev, recv_total);
}

/**
 * @brief UART DMA接收空闲中断处理函数
 * 
 * @param uart_id UART设备ID
 */
void uart_dmarx_idle_isr(uint8_t uart_id)
{
    uart_device_t *dev;
    uint16_t recv_total;

    if (!uart_id_valid(uart_id)) {
        return;
    }

    dev = &s_uart_dev[uart_id];
    if (!dev->inited) {
        return;
    }

    recv_total = uart_get_recv_total_from_dma(uart_id, dev);
    uart_commit_dmarx_bytes(dev, recv_total);
}

/**
 * @brief 获取UART诊断信息
 * 
 * @param uart_id UART设备ID
 * @param out_diag 输出诊断信息结构体指针
 */
void uart_get_diag(uint8_t uart_id, uart_diag_t *out_diag)
{
    if (out_diag == NULL) {
        return;
    }

    memset(out_diag, 0, sizeof(*out_diag));
    if (!uart_id_valid(uart_id)) {
        return;
    }

    *out_diag = s_uart_dev[uart_id].diag;
}

/**
 * @brief 清除UART诊断信息
 * 
 * @param uart_id UART设备ID
 */
void uart_clear_diag(uint8_t uart_id)
{
    if (!uart_id_valid(uart_id)) {
        return;
    }

    memset(&s_uart_dev[uart_id].diag, 0, sizeof(s_uart_dev[uart_id].diag));
}

/**
 * @brief 记录UART发送构建溢出次数
 * 
 * @param uart_id UART设备ID
 */
void uart_note_tx_build_oversize(uint8_t uart_id)
{
    if (!uart_id_valid(uart_id)) {
        return;
    }

    s_uart_dev[uart_id].diag.tx_build_oversize_cnt++;
}
