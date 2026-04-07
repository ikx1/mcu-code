#ifndef CAN_QUEUE_PORT_H
#define CAN_QUEUE_PORT_H

#include <stdint.h>
#include "can_irq_hooks.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Porting interface for CAN queue transport.
 * Implemented by MCU platform layer.
 */
uint8_t can_queue_port_init(void);
uint8_t can_queue_port_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc);
uint32_t can_queue_port_lock(void);
void can_queue_port_unlock(uint32_t key);
void can_queue_port_bind_irq_hooks(const can_irq_hooks_t *hooks);

#ifdef __cplusplus
}
#endif

#endif /* CAN_QUEUE_PORT_H */
