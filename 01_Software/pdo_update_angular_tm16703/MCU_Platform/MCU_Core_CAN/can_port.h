#ifndef CAN_PORT_H
#define CAN_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Platform CAN porting hooks.
 * Keep HAL-specific CAN init and TX details behind this interface.
 */
uint8_t can_port_init(void);
uint8_t can_port_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t can_port_recover(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PORT_H */
