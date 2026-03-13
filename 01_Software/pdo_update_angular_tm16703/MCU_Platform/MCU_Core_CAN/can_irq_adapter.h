#ifndef CAN_IRQ_ADAPTER_H
#define CAN_IRQ_ADAPTER_H

#include <stdint.h>
#include "can_irq_hooks.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t error_irq_count;
    uint32_t busoff_count;
    uint32_t recover_ok_count;
    uint32_t recover_fail_count;
    uint32_t last_error_code;
    uint8_t recover_pending;
} can_irq_diag_t;

/*
 * IRQ adapter entry points for CAN1 on STM32F103.
 * Keep CAN ISR glue in MCU platform layer.
 */
void mcu_can1_tx_irq_adapter(void);
void mcu_can1_rx0_irq_adapter(void);
void mcu_can1_sce_irq_adapter(void);
void mcu_can_irq_bind_hooks(const can_irq_hooks_t *hooks);

void mcu_can_diag_snapshot(can_irq_diag_t *out);
void mcu_can_diag_poll_recover(void);
void mcu_can_diag_reset_counters(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_IRQ_ADAPTER_H */
