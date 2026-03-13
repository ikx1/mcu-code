#include "can_port.h"

#include <stddef.h>

#include "can.h"

#include "irq_guard.h"

#include "stm32f10x_can.h"
#include "misc.h"

#define CAN_PORT_MAX_DLC 8u

static volatile uint8_t s_can_port_recovering = 0u;

static void can_port_enable_sce_irq(void)
{
    NVIC_InitTypeDef nvic = {0};

    CAN_ITConfig(CAN1, CAN_IT_ERR, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_BOF, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_EPV, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_EWG, ENABLE);
    CAN_ITConfig(CAN1, CAN_IT_LEC, ENABLE);

    nvic.NVIC_IRQChannel = CAN1_SCE_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5u;
    nvic.NVIC_IRQChannelSubPriority = 0u;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

uint8_t can_port_init(void)
{
    if (can_init() != 0u)
    {
        return 1u;
    }

    can_port_enable_sce_irq();
    return 0u;
}

uint8_t can_port_recover(void)
{
    uint32_t primask = mcu_irq_guard_lock();

    s_can_port_recovering = 1u;
    if (can_port_init() != 0u)
    {
        s_can_port_recovering = 0u;
        mcu_irq_guard_unlock(primask);
        return 1u;
    }

    s_can_port_recovering = 0u;
    mcu_irq_guard_unlock(primask);
    return 0u;
}

uint8_t can_port_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    CanTxMsg tx_msg = {0};
    uint8_t mailbox;
    uint8_t i;

    if (s_can_port_recovering != 0u)
    {
        return 3u;
    }

    if (data == NULL)
    {
        return 1u;
    }

    if (dlc > CAN_PORT_MAX_DLC)
    {
        dlc = CAN_PORT_MAX_DLC;
    }

    tx_msg.StdId = std_id;
    tx_msg.ExtId = 0u;
    tx_msg.IDE = CAN_ID_STD;
    tx_msg.RTR = CAN_RTR_DATA;
    tx_msg.DLC = dlc;
    for (i = 0u; i < CAN_PORT_MAX_DLC; i++)
    {
        tx_msg.Data[i] = (i < dlc) ? data[i] : 0u;
    }

    mailbox = CAN_Transmit(CAN1, &tx_msg);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 2u;
    }

    return 0u;
}
