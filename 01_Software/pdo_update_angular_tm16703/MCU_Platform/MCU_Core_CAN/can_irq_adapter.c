#include "can_irq_adapter.h"

#include <string.h>

#include "can_port.h"
#include "can_queue_port.h"
#include "irq_guard.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x_can.h"

#define CAN_RECOVER_RETRY_MS 100u
#define CAN_IRQ_MAX_DLC      8u

static can_irq_diag_t s_can_diag = {0};
static can_irq_hooks_t s_can_irq_hooks = {0};
static TickType_t s_can_last_recover_try_tick = 0u;
static uint8_t s_can_busoff_latched = 0u;

static uint32_t can_diag_lock(void)
{
    return mcu_irq_guard_lock();
}

static void can_diag_unlock(uint32_t primask)
{
    mcu_irq_guard_unlock(primask);
}

static can_irq_hooks_t can_irq_hooks_snapshot_task(void)
{
    uint32_t primask;
    can_irq_hooks_t hooks;

    primask = can_diag_lock();
    hooks = s_can_irq_hooks;
    can_diag_unlock(primask);

    return hooks;
}

static can_irq_hooks_t can_irq_hooks_snapshot_isr(void)
{
    return s_can_irq_hooks;
}

static void can_diag_record_busoff(uint32_t esr)
{
    s_can_diag.last_error_code = esr;

    if (s_can_busoff_latched == 0u)
    {
        s_can_busoff_latched = 1u;
        s_can_diag.busoff_count++;
    }

    s_can_diag.recover_pending = 1u;
}

static void can_diag_record_error(uint32_t esr)
{
    s_can_diag.error_irq_count++;
    s_can_diag.last_error_code = esr;

    if ((esr & CAN_ESR_BOFF) != 0u)
    {
        can_diag_record_busoff(esr);
    }
}

static void can_tx_complete_isr(void)
{
    can_irq_hooks_t hooks = can_irq_hooks_snapshot_isr();

    if (hooks.tx_complete_isr != NULL)
    {
        hooks.tx_complete_isr(hooks.ctx);
    }
}

void mcu_can1_tx_irq_adapter(void)
{
    if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
        can_tx_complete_isr();
    }
}

void mcu_can1_rx0_irq_adapter(void)
{
    can_irq_hooks_t hooks = can_irq_hooks_snapshot_isr();

    while (CAN_MessagePending(CAN1, CAN_FIFO0) > 0u)
    {
        CanRxMsg rx_msg = {0};
        uint8_t dlc;

        CAN_Receive(CAN1, CAN_FIFO0, &rx_msg);

        if ((rx_msg.IDE != CAN_ID_STD) || (rx_msg.RTR != CAN_RTR_DATA))
        {
            continue;
        }

        if (hooks.rx_frame_isr == NULL)
        {
            continue;
        }

        dlc = rx_msg.DLC;
        if (dlc > CAN_IRQ_MAX_DLC)
        {
            dlc = CAN_IRQ_MAX_DLC;
        }

        (void)hooks.rx_frame_isr(rx_msg.StdId, rx_msg.Data, dlc, hooks.ctx);
    }

    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

void mcu_can1_sce_irq_adapter(void)
{
    uint32_t primask;
    uint32_t esr = CAN1->ESR;

    primask = can_diag_lock();
    can_diag_record_error(esr);
    can_diag_unlock(primask);

    CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
    CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
    CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);
    CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);
    CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
}

void mcu_can_irq_bind_hooks(const can_irq_hooks_t *hooks)
{
    uint32_t primask;

    primask = can_diag_lock();
    if (hooks != NULL)
    {
        s_can_irq_hooks = *hooks;
    }
    else
    {
        memset(&s_can_irq_hooks, 0, sizeof(s_can_irq_hooks));
    }
    can_diag_unlock(primask);
}

void mcu_can_diag_snapshot(can_irq_diag_t *out)
{
    uint32_t primask;

    if (out == NULL)
    {
        return;
    }

    primask = can_diag_lock();
    *out = s_can_diag;
    can_diag_unlock(primask);
}

void mcu_can_diag_poll_recover(void)
{
    TickType_t now;
    TickType_t last_try_tick;
    uint8_t pending;
    uint8_t recovered = 0u;
    can_irq_hooks_t hooks;
    uint32_t primask;
    uint8_t recover_ret;

    primask = can_diag_lock();
    pending = s_can_diag.recover_pending;
    last_try_tick = s_can_last_recover_try_tick;
    can_diag_unlock(primask);

    if (pending == 0u)
    {
        return;
    }

    if (CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == RESET)
    {
        primask = can_diag_lock();
        if (s_can_diag.recover_pending != 0u)
        {
            s_can_diag.recover_pending = 0u;
            s_can_diag.recover_ok_count++;
            s_can_busoff_latched = 0u;
            recovered = 1u;
        }
        can_diag_unlock(primask);

        if (recovered != 0u)
        {
            hooks = can_irq_hooks_snapshot_task();
            if (hooks.bus_recovered != NULL)
            {
                hooks.bus_recovered(hooks.ctx);
            }
        }
        return;
    }

    now = xTaskGetTickCount();
    if ((now - last_try_tick) < pdMS_TO_TICKS(CAN_RECOVER_RETRY_MS))
    {
        return;
    }

    primask = can_diag_lock();
    s_can_last_recover_try_tick = now;
    can_diag_unlock(primask);

    recover_ret = can_port_recover();

    primask = can_diag_lock();
    if ((recover_ret == 0u) && (CAN_GetFlagStatus(CAN1, CAN_FLAG_BOF) == RESET))
    {
        s_can_diag.recover_pending = 0u;
        s_can_diag.recover_ok_count++;
        s_can_busoff_latched = 0u;
        recovered = 1u;
    }
    else
    {
        s_can_diag.recover_fail_count++;
    }
    can_diag_unlock(primask);

    if (recovered != 0u)
    {
        hooks = can_irq_hooks_snapshot_task();
        if (hooks.bus_recovered != NULL)
        {
            hooks.bus_recovered(hooks.ctx);
        }
    }
}

void mcu_can_diag_reset_counters(void)
{
    uint32_t primask;

    primask = can_diag_lock();
    memset(&s_can_diag, 0, sizeof(s_can_diag));
    s_can_last_recover_try_tick = 0u;
    s_can_busoff_latched = 0u;
    can_diag_unlock(primask);
}

uint8_t can_queue_port_init(void)
{
    return can_port_init();
}

uint8_t can_queue_port_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    return can_port_send_frame(std_id, data, dlc);
}

uint32_t can_queue_port_lock(void)
{
    return mcu_irq_guard_lock();
}

void can_queue_port_unlock(uint32_t key)
{
    mcu_irq_guard_unlock(key);
}

void can_queue_port_bind_irq_hooks(const can_irq_hooks_t *hooks)
{
    mcu_can_irq_bind_hooks(hooks);
}
