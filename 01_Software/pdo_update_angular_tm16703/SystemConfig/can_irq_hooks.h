#ifndef CAN_IRQ_HOOKS_H
#define CAN_IRQ_HOOKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    void (*tx_complete_isr)(void *ctx);
    uint8_t (*rx_frame_isr)(uint32_t std_id, const uint8_t *data, uint8_t dlc, void *ctx);
    void (*bus_recovered)(void *ctx);
    void *ctx;
} can_irq_hooks_t;

#ifdef __cplusplus
}
#endif

#endif /* CAN_IRQ_HOOKS_H */
