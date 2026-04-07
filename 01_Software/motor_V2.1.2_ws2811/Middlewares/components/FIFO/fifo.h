#ifndef FIFO_H
#define FIFO_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*lock_fun)(void); /* Compatibility placeholder; unused now. */

typedef struct
{
    uint8_t  *buf;
    uint16_t  size;
    volatile uint16_t head;
    volatile uint16_t tail;
} fifo_t;

void     fifo_register(fifo_t *pfifo, uint8_t *pfifo_buf, uint32_t size,
                       lock_fun lock, lock_fun unlock);
void     fifo_release(fifo_t *pfifo);
uint32_t fifo_write(fifo_t *pfifo, const uint8_t *pbuf, uint32_t size);
uint32_t fifo_read(fifo_t *pfifo, uint8_t *pbuf, uint32_t size);
uint32_t fifo_get_total_size(fifo_t *pfifo);
uint32_t fifo_get_free_size(fifo_t *pfifo);
uint32_t fifo_get_occupy_size(fifo_t *pfifo);

#endif /* FIFO_H */
