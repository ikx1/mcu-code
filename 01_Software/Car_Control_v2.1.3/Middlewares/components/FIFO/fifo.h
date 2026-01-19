#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdbool.h>
#include <stdint.h>

typedef void (*lock_fun)(void); /* 兼容老签名：不再使用 */

typedef struct
{
    uint8_t  *buf;              /* 缓冲区首地址 */
    uint16_t  size;             /* 缓冲区大小（必须是2的幂） */
    volatile uint16_t head;     /* 生产者写索引（仅生产者修改） */
    volatile uint16_t tail;     /* 消费者读索引（仅消费者修改） */
}_fifo_t;

/* API（保持不变） */
void     fifo_register(_fifo_t *pfifo, uint8_t *pfifo_buf, uint32_t size,
                       lock_fun lock, lock_fun unlock);
void     fifo_release(_fifo_t *pfifo);
uint32_t fifo_write(_fifo_t *pfifo, const uint8_t *pbuf, uint32_t size);
uint32_t fifo_read (_fifo_t *pfifo, uint8_t *pbuf, uint32_t size);
uint32_t fifo_get_total_size (_fifo_t *pfifo);
uint32_t fifo_get_free_size  (_fifo_t *pfifo);
uint32_t fifo_get_occupy_size(_fifo_t *pfifo);


#endif /* _FIFO_H_ */
