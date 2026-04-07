#include "can_port.h"

#include "can.h"

#define CAN_PORT_MAX_DLC 8u

typedef struct
{
    uint8_t tx_irq_enabled;
    uint8_t rx0_irq_enabled;
    uint8_t sce_irq_enabled;
} can_port_irq_state_t;

static volatile uint8_t s_can_port_recovering = 0u;

static uint32_t can_port_irq_notification_mask(void)
{
    return CAN_IT_RX_FIFO0_MSG_PENDING |
           CAN_IT_RX_FIFO0_FULL |
           CAN_IT_RX_FIFO0_OVERRUN |
           CAN_IT_TX_MAILBOX_EMPTY |
           CAN_IT_ERROR_WARNING |
           CAN_IT_ERROR_PASSIVE |
           CAN_IT_BUSOFF |
           CAN_IT_LAST_ERROR_CODE |
           CAN_IT_ERROR;
}

static void can_port_irq_save_disable(can_port_irq_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->tx_irq_enabled = (uint8_t)(NVIC_GetEnableIRQ(CAN1_TX_IRQn) != 0u);
    state->rx0_irq_enabled = (uint8_t)(NVIC_GetEnableIRQ(CAN1_RX0_IRQn) != 0u);
    state->sce_irq_enabled = (uint8_t)(NVIC_GetEnableIRQ(CAN1_SCE_IRQn) != 0u);

    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
}

static void can_port_irq_restore(const can_port_irq_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    if (state->tx_irq_enabled != 0u)
    {
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    }

    if (state->rx0_irq_enabled != 0u)
    {
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    }

    if (state->sce_irq_enabled != 0u)
    {
        HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    }
}

uint8_t can_port_init(void)
{
    CAN_FilterTypeDef can_filter_st = {0};

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
    {
        return 1;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return 2;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, can_port_irq_notification_mask()) != HAL_OK)
    {
        return 3;
    }

    return 0;
}

uint8_t can_port_recover(void)
{
    can_port_irq_state_t irq_state = {0};
    uint8_t ret = 0u;

    s_can_port_recovering = 1u;
    can_port_irq_save_disable(&irq_state);

    (void)HAL_CAN_Stop(&hcan1);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        ret = 1u;
        goto can_port_recover_exit;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, can_port_irq_notification_mask()) != HAL_OK)
    {
        ret = 2u;
        goto can_port_recover_exit;
    }

    if (HAL_CAN_ResetError(&hcan1) != HAL_OK)
    {
        ret = 3u;
        goto can_port_recover_exit;
    }

can_port_recover_exit:
    can_port_irq_restore(&irq_state);
    s_can_port_recovering = 0u;
    return ret;
}

uint8_t can_port_send_frame(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox = 0u;

    if (s_can_port_recovering != 0u)
    {
        return 3u;
    }

    if (data == NULL)
    {
        return 1;
    }

    if (dlc > CAN_PORT_MAX_DLC)
    {
        dlc = CAN_PORT_MAX_DLC;
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &mailbox) == HAL_OK) ? 0u : 2u;
}
