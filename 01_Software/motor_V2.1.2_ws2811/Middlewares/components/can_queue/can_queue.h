/**
 * @file can_queue.h
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __CAN_QUEUE_H__
#define __CAN_QUEUE_H__

/********************************** Includes *********************************/
#include <stdint.h>

/********************************** Defines **********************************/
#define CAN_QUEUE_NUM 32
#define CAN_RX_FIFO_SIZE 32
#define CAN_FRAME_MAX_DATA_LEN 8

 
/********************************** Variables ********************************/
typedef struct
{
    uint32_t std_id;
    uint8_t dlc;
    uint8_t data[CAN_FRAME_MAX_DATA_LEN];
} can_std_frame_t;

/********************************** Functions ********************************/
uint8_t can_queue_init(void);
uint8_t can_queue_push_frame(const can_std_frame_t *frame);
uint8_t can_queue_kick_tx(void);

/* RX FIFO API: task context reads. */
uint8_t can_rx_fifo_pop(can_std_frame_t *frame);

/* Optional diagnostics APIs for runtime observability. */
uint8_t can_tx_queue_pending_count_get(void);
uint8_t can_rx_fifo_count_get(void);
uint32_t can_rx_fifo_drop_count_get(void);
void can_rx_fifo_drop_count_clear(void);

#endif /* __CAN_QUEUE_H__ */
