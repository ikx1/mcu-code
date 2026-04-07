#include "can_irq_adapter.h"

#include "can.h"
#include "can_port.h"
#include "can_queue_port.h"
#include "irq_guard.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define CAN_RECOVER_RETRY_MS 100u
#define CAN_IRQ_MAX_DLC 8u

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
    /* Bind side updates hooks under global IRQ lock, so ISR read can stay lock-free. */
    return s_can_irq_hooks;
}

static void can_diag_record_error(uint32_t error_code)
{
    s_can_diag.error_irq_count++;
    s_can_diag.last_error_code = error_code;

    if ((error_code & HAL_CAN_ERROR_BOF) != 0u)
    {
        s_can_diag.busoff_count++;
        s_can_diag.recover_pending = 1u;
        s_can_busoff_latched = 1u;
    }
}

static void can_diag_poll_error_state(void)
{
    uint32_t error_code;
    uint8_t busoff_flag;
    uint32_t primask;

    error_code = HAL_CAN_GetError(&hcan1);
    busoff_flag = (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) != RESET) ? 1u : 0u;

    primask = can_diag_lock();

    if (error_code != HAL_CAN_ERROR_NONE)
    {
        if (error_code != s_can_diag.last_error_code)
        {
            s_can_diag.error_irq_count++;
        }
        s_can_diag.last_error_code = error_code;
    }

    if ((busoff_flag != 0u) && (s_can_busoff_latched == 0u))
    {
        s_can_busoff_latched = 1u;
        s_can_diag.busoff_count++;
        s_can_diag.recover_pending = 1u;
    }
    else if (busoff_flag == 0u)
    {
        s_can_busoff_latched = 0u;
    }

    can_diag_unlock(primask);
}

static void can_tx_complete_isr(CAN_HandleTypeDef *hcan)
{
    can_irq_hooks_t hooks;

    if ((hcan != NULL) && (hcan->Instance == CAN1))
    {
        hooks = can_irq_hooks_snapshot_isr();
        if (hooks.tx_complete_isr != NULL)
        {
            hooks.tx_complete_isr(hooks.ctx);
        }
    }
}

void mcu_can1_tx_irq_adapter(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void mcu_can1_rx0_irq_adapter(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void mcu_can1_sce_irq_adapter(void)
{
    HAL_CAN_IRQHandler(&hcan1);
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
    uint8_t pending = 0u;
    uint8_t recovered = 0u;
    TickType_t last_try_tick = 0u;
    uint32_t primask;
    uint8_t recover_ret;

    can_diag_poll_error_state();

    primask = can_diag_lock();
    pending = s_can_diag.recover_pending;
    last_try_tick = s_can_last_recover_try_tick;
    can_diag_unlock(primask);

    if (pending == 0u)
    {
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
    if (recover_ret == 0u)
    {
        s_can_diag.recover_ok_count++;
        s_can_diag.recover_pending = 0u;
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
        can_irq_hooks_t hooks = can_irq_hooks_snapshot_task();
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

    (void)HAL_CAN_ResetError(&hcan1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CAN_IRQ_MAX_DLC];
    uint8_t dlc = 0u;
    uint32_t pending;

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    while (pending > 0u)
    {
        memset(rx_data, 0, sizeof(rx_data));
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA))
        {
            can_irq_hooks_t hooks = can_irq_hooks_snapshot_isr();

            dlc = rx_header.DLC;
            if (dlc > CAN_IRQ_MAX_DLC)
            {
                dlc = CAN_IRQ_MAX_DLC;
            }

            if (hooks.rx_frame_isr != NULL)
            {
                (void)hooks.rx_frame_isr(rx_header.StdId, rx_data, dlc, hooks.ctx);
            }
        }

        pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_complete_isr(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_complete_isr(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    can_tx_complete_isr(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error_code;

    if ((hcan == NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    error_code = HAL_CAN_GetError(hcan);
    can_diag_record_error(error_code);
    (void)HAL_CAN_ResetError(hcan);
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
