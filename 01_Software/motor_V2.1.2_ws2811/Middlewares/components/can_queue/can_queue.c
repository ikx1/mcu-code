/**
 * @file can_queue.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "can_queue.h"
#include "can_queue_port.h"
#include <string.h>

/********************************** Defines **********************************/
typedef enum
{
    CAN_QUEUE_IDLE = 0,
    CAN_QUEUE_BUSY
} can_queue_state_t;

typedef struct
{
    can_std_frame_t can_tx_str[CAN_QUEUE_NUM];
    uint8_t head;
    uint8_t tail;
    can_queue_state_t state;
} can_tx_queue_t;

typedef struct
{
    can_std_frame_t buffer[CAN_RX_FIFO_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} can_rx_fifo_t;


/********************************** Variables ********************************/
/* Internal CAN TX queue and RX FIFO instances. */

static can_tx_queue_t canbus_queue = {0};
static can_rx_fifo_t can_rx_fifo = {0};
static volatile uint32_t can_rx_fifo_drop_count = 0u;

/********************************** Functions ********************************/
static uint32_t can_queue_irq_lock(void)
{
    return can_queue_port_lock();
}

static void can_queue_irq_unlock(uint32_t primask)
{
    can_queue_port_unlock(primask);
}

static uint8_t can_queue_start_send_idle(can_tx_queue_t* queue_p);
static uint8_t can_queue_delete_head(can_tx_queue_t* queue_p);
static uint8_t can_queue_normalize_dlc(uint8_t dlc);
static uint8_t can_queue_tx_complete_isr(void);
static uint8_t can_queue_on_bus_recovered(void);
static uint8_t can_rx_fifo_push_isr(const can_std_frame_t *frame);
static void can_queue_tx_complete_hook(void *ctx);
static uint8_t can_queue_rx_frame_hook(uint32_t std_id, const uint8_t *data, uint8_t dlc, void *ctx);
static void can_queue_bus_recovered_hook(void *ctx);

uint8_t can_queue_init(void)
{
    can_irq_hooks_t irq_hooks = {0};
    uint8_t init_ret = 0u;
    uint32_t primask = can_queue_irq_lock();
    memset(&canbus_queue, 0, sizeof(canbus_queue));
    memset(&can_rx_fifo, 0, sizeof(can_rx_fifo));
    can_rx_fifo_drop_count = 0u;
    can_queue_irq_unlock(primask);

    irq_hooks.tx_complete_isr = can_queue_tx_complete_hook;
    irq_hooks.rx_frame_isr = can_queue_rx_frame_hook;
    irq_hooks.bus_recovered = can_queue_bus_recovered_hook;
    irq_hooks.ctx = NULL;
    /* Bind first so early CAN IRQs after init can be consumed by queue hooks. */
    can_queue_port_bind_irq_hooks(&irq_hooks);

    init_ret = can_queue_port_init();
    if (init_ret != 0u)
    {
        can_queue_port_bind_irq_hooks(NULL);
    }

    return init_ret;
}

static uint8_t can_queue_normalize_dlc(uint8_t dlc)
{
    if (dlc > CAN_FRAME_MAX_DATA_LEN)
    {
        return CAN_FRAME_MAX_DATA_LEN;
    }

    return dlc;
}

static void can_queue_tx_complete_hook(void *ctx)
{
    (void)ctx;
    (void)can_queue_tx_complete_isr();
}

static uint8_t can_queue_rx_frame_hook(uint32_t std_id, const uint8_t *data, uint8_t dlc, void *ctx)
{
    can_std_frame_t frame = {0};

    (void)ctx;

    if (data == NULL)
    {
        return 0u;
    }

    frame.std_id = std_id;
    frame.dlc = can_queue_normalize_dlc(dlc);
    memcpy(frame.data, data, frame.dlc);

    return can_rx_fifo_push_isr(&frame);
}

static void can_queue_bus_recovered_hook(void *ctx)
{
    (void)ctx;
    (void)can_queue_on_bus_recovered();
}

uint8_t can_queue_push_frame(const can_std_frame_t *frame)
{
    uint8_t ret = 0;
    uint8_t start_send = 0;
    uint32_t primask = 0u;
    can_std_frame_t tx_frame = {0};
    can_std_frame_t frame_local = {0};

    if (frame == NULL)
    {
        return 1;
    }

    frame_local = *frame;
    frame_local.dlc = can_queue_normalize_dlc(frame_local.dlc);

    primask = can_queue_irq_lock();

    if ((canbus_queue.head + 1) % CAN_QUEUE_NUM == canbus_queue.tail)
    {
        ret = 1;
    }
    else
    {
        canbus_queue.can_tx_str[canbus_queue.head] = frame_local;
        canbus_queue.head = (canbus_queue.head + 1) % CAN_QUEUE_NUM;

        if (canbus_queue.state == CAN_QUEUE_IDLE)
        {
            canbus_queue.state = CAN_QUEUE_BUSY;
            tx_frame = canbus_queue.can_tx_str[canbus_queue.tail];
            start_send = 1u;
        }
    }

    can_queue_irq_unlock(primask);

    if ((ret == 0u) && (start_send != 0u))
    {
        ret = can_queue_port_send_frame(tx_frame.std_id, tx_frame.data, tx_frame.dlc);
        if (ret != 0u)
        {
            primask = can_queue_irq_lock();
            canbus_queue.state = CAN_QUEUE_IDLE;
            can_queue_irq_unlock(primask);
        }
    }

    return ret;
}

static uint8_t can_queue_delete_head(can_tx_queue_t* queue_p)
{
    if (queue_p == NULL)
    {
        return 1;
    }

    if (queue_p->head != queue_p->tail)
    {
        queue_p->tail = (queue_p->tail + 1) % CAN_QUEUE_NUM;
        return 0;
    }

    return 1;
}

static uint8_t can_queue_start_send_idle(can_tx_queue_t* queue_p)
{
    uint8_t ret = 0;

    if (queue_p == NULL)
    {
        return 1;
    }

    if ((queue_p->head != queue_p->tail) && (queue_p->state == CAN_QUEUE_IDLE))
    {
        queue_p->state = CAN_QUEUE_BUSY;
        ret = can_queue_port_send_frame(queue_p->can_tx_str[queue_p->tail].std_id,
                                        queue_p->can_tx_str[queue_p->tail].data,
                                        queue_p->can_tx_str[queue_p->tail].dlc);
        if (ret != 0u)
        {
            queue_p->state = CAN_QUEUE_IDLE;
        }

        return ret;
    }

    return 1;
}

static uint8_t can_queue_tx_complete_isr(void)
{
    if (can_queue_delete_head(&canbus_queue) != 0u)
    {
        canbus_queue.state = CAN_QUEUE_IDLE;
        return 1;
    }

    canbus_queue.state = CAN_QUEUE_IDLE;
    if (canbus_queue.head != canbus_queue.tail)
    {
        return can_queue_start_send_idle(&canbus_queue);
    }

    return 0;
}

uint8_t can_queue_kick_tx(void)
{
    uint8_t ret = 0u;
    uint8_t start_send = 0u;
    uint32_t primask = 0u;
    can_std_frame_t tx_frame = {0};

    primask = can_queue_irq_lock();
    if ((canbus_queue.head != canbus_queue.tail) && (canbus_queue.state == CAN_QUEUE_IDLE))
    {
        canbus_queue.state = CAN_QUEUE_BUSY;
        tx_frame = canbus_queue.can_tx_str[canbus_queue.tail];
        start_send = 1u;
    }
    can_queue_irq_unlock(primask);

    if (start_send == 0u)
    {
        return 0u;
    }

    ret = can_queue_port_send_frame(tx_frame.std_id, tx_frame.data, tx_frame.dlc);
    if (ret != 0u)
    {
        primask = can_queue_irq_lock();
        canbus_queue.state = CAN_QUEUE_IDLE;
        can_queue_irq_unlock(primask);
        return 1u;
    }

    return 0u;
}

static uint8_t can_queue_on_bus_recovered(void)
{
    uint32_t primask = can_queue_irq_lock();
    canbus_queue.state = CAN_QUEUE_IDLE;
    can_queue_irq_unlock(primask);

    return can_queue_kick_tx();
}

static uint8_t can_rx_fifo_push_isr(const can_std_frame_t *frame)
{
    uint8_t head = 0u;
    can_std_frame_t frame_local = {0};

    if (frame == NULL)
    {
        return 0;
    }

    frame_local = *frame;
    frame_local.dlc = can_queue_normalize_dlc(frame_local.dlc);

    if (can_rx_fifo.count >= CAN_RX_FIFO_SIZE)
    {
        if (can_rx_fifo_drop_count < 0xFFFFFFFFu)
        {
            can_rx_fifo_drop_count++;
        }
        return 0;
    }

    head = can_rx_fifo.head;
    can_rx_fifo.buffer[head] = frame_local;
    can_rx_fifo.head = (uint8_t)((head + 1u) % CAN_RX_FIFO_SIZE);
    can_rx_fifo.count++;

    return 1;
}

uint8_t can_rx_fifo_pop(can_std_frame_t *frame)
{
    uint8_t ret = 0;
    uint32_t primask = 0u;

    if (frame == NULL)
    {
        return 0;
    }

    primask = can_queue_irq_lock();

    if (can_rx_fifo.count > 0)
    {
        uint8_t tail = can_rx_fifo.tail;
        *frame = can_rx_fifo.buffer[tail];
        can_rx_fifo.tail = (uint8_t)((tail + 1u) % CAN_RX_FIFO_SIZE);
        can_rx_fifo.count--;
        ret = 1;
    }

    can_queue_irq_unlock(primask);

    return ret;
}

uint8_t can_tx_queue_pending_count_get(void)
{
    uint8_t head;
    uint8_t tail;
    uint8_t pending;
    uint32_t primask = can_queue_irq_lock();

    head = canbus_queue.head;
    tail = canbus_queue.tail;
    if (head >= tail)
    {
        pending = (uint8_t)(head - tail);
    }
    else
    {
        pending = (uint8_t)(CAN_QUEUE_NUM - (uint8_t)(tail - head));
    }

    can_queue_irq_unlock(primask);
    return pending;
}

uint8_t can_rx_fifo_count_get(void)
{
    uint8_t count;
    uint32_t primask = can_queue_irq_lock();
    count = can_rx_fifo.count;
    can_queue_irq_unlock(primask);
    return count;
}

uint32_t can_rx_fifo_drop_count_get(void)
{
    uint32_t drop_count;
    uint32_t primask = can_queue_irq_lock();
    drop_count = can_rx_fifo_drop_count;
    can_queue_irq_unlock(primask);
    return drop_count;
}

void can_rx_fifo_drop_count_clear(void)
{
    uint32_t primask = can_queue_irq_lock();
    can_rx_fifo_drop_count = 0u;
    can_queue_irq_unlock(primask);
}
