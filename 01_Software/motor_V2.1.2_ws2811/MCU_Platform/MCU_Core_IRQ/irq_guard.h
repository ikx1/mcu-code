#ifndef IRQ_GUARD_H
#define IRQ_GUARD_H

#include <stdint.h>

#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t mcu_irq_guard_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void mcu_irq_guard_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#ifdef __cplusplus
}
#endif

#endif /* IRQ_GUARD_H */
