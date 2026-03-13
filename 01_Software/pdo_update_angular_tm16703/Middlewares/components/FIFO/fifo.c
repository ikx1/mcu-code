#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fifo.h"

#ifndef FIFO_DMB
#if defined(__ARMCC_VERSION) || defined(__CC_ARM) || defined(__ARMCC_VERSION_MAJOR)
#define FIFO_DMB() __dmb(0)
#elif defined(__GNUC__)
#define FIFO_DMB() __asm volatile ("dmb" ::: "memory")
#else
#define FIFO_DMB() do {} while (0)
#endif
#endif

/* Return nearest power-of-two less than or equal to x. */
static uint32_t floor_pow2(uint32_t x)
{
    if (x < 2u) {
        return 1u;
    }

    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return (x + 1u) >> 1;
}

void fifo_register(fifo_t *pfifo, uint8_t *pfifo_buf, uint32_t size,
                   lock_fun lock, lock_fun unlock)
{
    (void)lock;
    (void)unlock;

    if (pfifo == NULL || pfifo_buf == NULL || size < 2u) {
        return;
    }

    uint32_t s = floor_pow2(size);
    if (s < 2u) {
        s = 2u;
    }
    if (s > 0x8000u) {
        s = 0x8000u;
    }

    pfifo->buf = pfifo_buf;
    pfifo->size = (uint16_t)s;
    pfifo->head = 0u;
    pfifo->tail = 0u;
}

void fifo_release(fifo_t *pfifo)
{
    if (pfifo == NULL) {
        return;
    }

    pfifo->buf = NULL;
    pfifo->size = 0u;
    pfifo->head = 0u;
    pfifo->tail = 0u;
}

/* Single producer write path (ISR or task, not both at once). */
uint32_t fifo_write(fifo_t *pfifo, const uint8_t *pbuf, uint32_t n)
{
    if (pfifo == NULL || pfifo->buf == NULL || pbuf == NULL || n == 0u) {
        return 0u;
    }

    uint16_t size = pfifo->size;
    uint16_t mask = (uint16_t)(size - 1u);
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;

    uint16_t free = (uint16_t)((tail - head - 1u) & mask);
    uint16_t wr = (n > free) ? free : (uint16_t)n;
    if (wr == 0u) {
        return 0u;
    }

    uint16_t first = (uint16_t)((size - (head & mask)) & mask);
    if (first > wr) {
        first = wr;
    }
    memcpy(&pfifo->buf[head & mask], pbuf, first);

    uint16_t remain = (uint16_t)(wr - first);
    if (remain != 0u) {
        memcpy(&pfifo->buf[0], pbuf + first, remain);
    }

    /* Publish head after payload writes are visible. */
    FIFO_DMB();
    pfifo->head = (uint16_t)((head + wr) & mask);
    return wr;
}

/* Single consumer read path (task context). */
uint32_t fifo_read(fifo_t *pfifo, uint8_t *pbuf, uint32_t n)
{
    if (pfifo == NULL || pfifo->buf == NULL || pbuf == NULL || n == 0u) {
        return 0u;
    }

    uint16_t size = pfifo->size;
    uint16_t mask = (uint16_t)(size - 1u);
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;

    uint16_t avail = (uint16_t)((head - tail) & mask);
    uint16_t rd = (n > avail) ? avail : (uint16_t)n;
    if (rd == 0u) {
        return 0u;
    }

    uint16_t first = (uint16_t)((size - (tail & mask)) & mask);
    if (first > rd) {
        first = rd;
    }
    memcpy(pbuf, &pfifo->buf[tail & mask], first);

    uint16_t remain = (uint16_t)(rd - first);
    if (remain != 0u) {
        memcpy(pbuf + first, &pfifo->buf[0], remain);
    }

    pfifo->tail = (uint16_t)((tail + rd) & mask);
    return rd;
}

uint32_t fifo_get_total_size(fifo_t *pfifo)
{
    return (pfifo != NULL && pfifo->buf != NULL) ? pfifo->size : 0u;
}

uint32_t fifo_get_free_size(fifo_t *pfifo)
{
    if (pfifo == NULL || pfifo->buf == NULL) {
        return 0u;
    }

    uint16_t mask = (uint16_t)(pfifo->size - 1u);
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;
    return (uint16_t)((tail - head - 1u) & mask);
}

uint32_t fifo_get_occupy_size(fifo_t *pfifo)
{
    if (pfifo == NULL || pfifo->buf == NULL) {
        return 0u;
    }

    uint16_t mask = (uint16_t)(pfifo->size - 1u);
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;
    return (uint16_t)((head - tail) & mask);
}
