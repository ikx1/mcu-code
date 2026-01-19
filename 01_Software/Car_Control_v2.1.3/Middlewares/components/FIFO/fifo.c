#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "fifo.h"

#include "main.h"

// /* 某些编译器宏 */
// #ifndef __DMB
//   #if defined(__ARMCC_VERSION) || defined(__CC_ARM) || defined(__ARMCC_VERSION_MAJOR)
//     #define __DMB() __dmb(0)
//   #elif defined(__GNUC__)
//     #define __DMB() __asm volatile ("dmb" ::: "memory")
//   #else
//     #define __DMB() do{}while(0)
//   #endif
// #endif

/* 取最近的2次幂（向下取），也可以选择直接断言 size 必须是2的幂 */
static uint32_t floor_pow2(uint32_t x)
{
    if (x < 2) return 1;
    /* 抹高法 */
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return (x + 1) >> 1;
}

void fifo_register(_fifo_t *pfifo, uint8_t *pfifo_buf, uint32_t size,
                   lock_fun lock, lock_fun unlock)
{
    (void)lock; (void)unlock; /* 不再使用，保持兼容 */
    if (!pfifo || !pfifo_buf || size < 2)
        return;

    /* 建议调用处直接传2的幂；为了兼容，这里做一次向下取2次幂 */
    uint32_t s = floor_pow2(size);
    if (s < 2) s = 2;
    if (s > 0xFFFF) s = 0x10000u; /* 但我们用 uint16_t 索引，实际最大 65536 */

    pfifo->buf  = pfifo_buf;
    pfifo->size = (uint16_t)s;
    pfifo->head = 0;
    pfifo->tail = 0;
}

void fifo_release(_fifo_t *pfifo)
{
    if (!pfifo) return;
    pfifo->buf  = NULL;
    pfifo->size = 0;
    pfifo->head = 0;
    pfifo->tail = 0;
}

/* 生产者写：单写者上下文（ISR 或 任务二选一） */
uint32_t fifo_write(_fifo_t *pfifo, const uint8_t *pbuf, uint32_t n)
{
    if (!pfifo || !pfifo->buf || !pbuf || n == 0) return 0;

    uint16_t size = pfifo->size;
    uint16_t mask = size - 1;
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;

    uint16_t free = (uint16_t)((tail - head - 1) & mask);
    uint16_t wr   = (n > free) ? free : (uint16_t)n;
    if (wr == 0) return 0;

    uint16_t first = (uint16_t)((size - (head & mask)) & mask);
    if (first > wr) first = wr;
    memcpy(&pfifo->buf[head & mask], pbuf, first);

    uint16_t remain = wr - first;
    if (remain)
        memcpy(&pfifo->buf[0], pbuf + first, remain);

    /* 先写数据，再发布 head（内存屏障保证顺序） */
    __DMB();
    pfifo->head = (uint16_t)((head + wr) & mask);
    return wr;
}

/* 消费者读：单读者上下文（与写者不同上下文） */
uint32_t fifo_read(_fifo_t *pfifo, uint8_t *pbuf, uint32_t n)
{
    if (!pfifo || !pfifo->buf || !pbuf || n == 0) return 0;

    uint16_t size = pfifo->size;
    uint16_t mask = size - 1;
    uint16_t head = pfifo->head;
    uint16_t tail = pfifo->tail;

    uint16_t avail = (uint16_t)((head - tail) & mask);
    uint16_t rd    = (n > avail) ? avail : (uint16_t)n;
    if (rd == 0) return 0;

    uint16_t first = (uint16_t)((size - (tail & mask)) & mask);
    if (first > rd) first = rd;
    memcpy(pbuf, &pfifo->buf[tail & mask], first);

    uint16_t remain = rd - first;
    if (remain)
        memcpy(pbuf + first, &pfifo->buf[0], remain);

    /* 读完再推进 tail */
    pfifo->tail = (uint16_t)((tail + rd) & mask);
    return rd;
}

uint32_t fifo_get_total_size(_fifo_t *pfifo)
{
    return (pfifo && pfifo->buf) ? pfifo->size : 0;
}

uint32_t fifo_get_free_size(_fifo_t *pfifo)
{
    if (!pfifo || !pfifo->buf) return 0;
    uint16_t mask = pfifo->size - 1;
    uint16_t head = pfifo->head, tail = pfifo->tail;
    return (uint16_t)((tail - head - 1) & mask);
}

uint32_t fifo_get_occupy_size(_fifo_t *pfifo)
{
    if (!pfifo || !pfifo->buf) return 0;
    uint16_t mask = pfifo->size - 1;
    uint16_t head = pfifo->head, tail = pfifo->tail;
    return (uint16_t)((head - tail) & mask);
}

///* 可选：快速查询（无拷贝） */
//static inline uint32_t fifo_peek_avail(_fifo_t *pfifo)
//{
//    uint16_t head = pfifo->head, tail = pfifo->tail;
//    return (uint16_t)((head - tail) & (pfifo->size - 1));
//}
