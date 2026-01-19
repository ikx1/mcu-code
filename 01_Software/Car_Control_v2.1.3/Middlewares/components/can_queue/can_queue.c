#include "can_queue.h"

/*===================== 静态检查：必须 2^n =====================*/
#define CANQ_MASK_TX   (CAN_QUEUE_NUM - 1U)
#define CANQ_MASK_RX   (CAN_RX_FIFO_SIZE - 1U)

#if (CAN_QUEUE_NUM & CANQ_MASK_TX)
#error "CAN_QUEUE_NUM must be power of two"
#endif
#if (CAN_RX_FIFO_SIZE & CANQ_MASK_RX)
#error "CAN_RX_FIFO_SIZE must be power of two"
#endif

CANBUS_QUEUE_INFO g_can_txq = {0};
CAN_RxFIFO_t      g_can_rxfifo = {0};

/*===================== 临界区（短小：仅保护 head/tail 等） =====================*/
static inline uint32_t canq_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void canq_exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static inline uint16_t next_tx(uint16_t i) { return (uint16_t)((i + 1U) & CANQ_MASK_TX); }
static inline uint16_t next_rx(uint16_t i) { return (uint16_t)((i + 1U) & CANQ_MASK_RX); }

/*===================== Filter/Start/Notify =====================*/
void can_filter_init_all_pass(void)
{
    CAN_FilterTypeDef f = {0};

    f.FilterActivation = ENABLE;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;

    f.FilterIdHigh = 0x0000;
    f.FilterIdLow  = 0x0000;
    f.FilterMaskIdHigh = 0x0000;
    f.FilterMaskIdLow  = 0x0000;

    /* F429 有 CAN2：必须设置 SlaveStartFilterBank 做分配 */
    f.FilterBank = 0;                 /* CAN1 用 0~13 */
    f.SlaveStartFilterBank = 14;      /* CAN2 从 14 开始 */
    f.FilterFIFOAssignment = CAN_RX_FIFO0;

    (void)HAL_CAN_ConfigFilter(&hcan1, &f);
}

static void can_hw_start_and_notify(void)
{
    (void)HAL_CAN_Start(&hcan1);

    /* TX/RX/ERROR 通知都开了：长稳必备 */
    (void)HAL_CAN_ActivateNotification(&hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY     |
        CAN_IT_ERROR                |
        CAN_IT_BUSOFF               |
        CAN_IT_LAST_ERROR_CODE
    );
}

static void can_hw_recover(CANBUS_QUEUE_INFO *q)
{
	(void)HAL_CAN_Stop(&hcan1);
	can_filter_init_all_pass();
	can_hw_start_and_notify();

	if (q)
	{
		q->kick_lock = 0U;
		q->recover_cnt++;
		q->last_progress_ms = HAL_GetTick();
	}
}

void can_queue_init(void)
{
    uint32_t pm = canq_enter_critical();
    g_can_txq.head = g_can_txq.tail = 0;
    g_can_txq.kick_lock = 0;
    g_can_txq.drop_cnt = 0;
    g_can_txq.replace_cnt = 0;
    g_can_txq.addtx_fail_cnt = 0;
    g_can_txq.kick_cnt = 0;
    g_can_txq.busoff_cnt = 0;
    g_can_txq.recover_cnt = 0;
	g_can_txq.stall_recover_cnt = 0;
	g_can_txq.kick_unlock_cnt = 0;
    g_can_txq.last_err = 0;
    g_can_txq.busoff_pending = 0;
	g_can_txq.last_kick_ms = 0;
	g_can_txq.last_progress_ms = 0;
	g_can_txq.last_isr_tx_ms = 0;

    g_can_rxfifo.head = g_can_rxfifo.tail = g_can_rxfifo.count = 0;
    g_can_rxfifo.overflow_cnt = 0;
    canq_exit_critical(pm);

    can_filter_init_all_pass();
    can_hw_start_and_notify();
}

/*===================== kick lock =====================*/
static inline uint8_t try_lock_kick(CANBUS_QUEUE_INFO *q)
{
    uint8_t ok = 0;
    uint32_t pm = canq_enter_critical();
    if (q->kick_lock == 0U) { q->kick_lock = 1U; ok = 1U; }
    canq_exit_critical(pm);
    return ok;
}
static inline void unlock_kick(CANBUS_QUEUE_INFO *q)
{
    uint32_t pm = canq_enter_critical();
    q->kick_lock = 0U;
    canq_exit_critical(pm);
}

/*===================== 发送推进：尽量塞满 3 邮箱 =====================*/
void can_tx_kick(CANBUS_QUEUE_INFO *q)
{
    if (!q) return;
    if (!try_lock_kick(q)) return;

    q->kick_cnt++;
	q->last_kick_ms = HAL_GetTick();

    while (q->head != q->tail)
    {
        uint32_t free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
        if (free == 0U) break;

        uint32_t mbox = 0;
        CAN_TxItem_t *it = &q->buf[q->tail];

        if (HAL_CAN_AddTxMessage(&hcan1, &it->header, it->data, &mbox) != HAL_OK)
        {
            q->addtx_fail_cnt++;
	        q->last_err = HAL_CAN_GetError(&hcan1);
            if (q->last_err & HAL_CAN_ERROR_BOF)
            {
	            q->busoff_pending = 1U;
            }
            break;
        }

        /* 只要成功进入硬件邮箱，就释放这个软件元素 */
        q->tail = next_tx(q->tail);
	    q->last_progress_ms = HAL_GetTick();
    }

    unlock_kick(q);
}

/*===================== 覆盖策略：扫描未入邮箱帧 =====================*/
static uint8_t replace_unsent_by_id(CANBUS_QUEUE_INFO *q,
                                   const CAN_TxHeaderTypeDef *h,
                                   const uint8_t *d8)
{
    uint16_t idx = q->tail;
    while (idx != q->head)
    {
        CAN_TxHeaderTypeDef *eh = &q->buf[idx].header;
        if (eh->StdId == h->StdId &&
            eh->IDE   == h->IDE   &&
            eh->RTR   == h->RTR   &&
            eh->DLC   == h->DLC)
        {
            *eh = *h;
            memcpy(q->buf[idx].data, d8, 8);
            q->replace_cnt++;
            return 1;
        }
        idx = next_tx(idx);
    }
    return 0;
}

/*===================== 入队（带：满队列抢救 kick + 重试） =====================*/
uint8_t can_tx_enqueue_ex(CANBUS_QUEUE_INFO *q,
                          const CAN_TxHeaderTypeDef *h,
                          const uint8_t *d8,
                          CAN_TxPolicy_t policy)
{
    if (!q || !h || !d8) return 2;
	uint8_t was_empty = 0U;

    /* REPLACE：先试图覆盖未发送帧 */
    uint32_t pm = canq_enter_critical();
    if (policy == CAN_TXQ_REPLACE_BY_ID)
    {
        if (replace_unsent_by_id(q, h, d8))
        {
            canq_exit_critical(pm);
            can_tx_kick(q);
            return 0;
        }
        policy = CAN_TXQ_DROP_OLD; /* 没找到就按实时帧处理 */
    }

	was_empty = (q->head == q->tail);

    uint16_t next = next_tx(q->head);
    if (next == q->tail)
    {
        /* 关键：队列满时先退出临界区抢救 kick，再回来重试一次 */
        canq_exit_critical(pm);
        can_tx_kick(q);

        pm = canq_enter_critical();
        next = next_tx(q->head);
        if (next == q->tail)
        {
            if (policy == CAN_TXQ_DROP_OLD)
            {
                q->tail = next_tx(q->tail); /* 丢最老，腾位 */
            }
            else
            {
                q->drop_cnt++;
                canq_exit_critical(pm);
                return 1; /* full 丢新 */
            }
        }
    }

    q->buf[q->head].header = *h;
    memcpy(q->buf[q->head].data, d8, 8);

    __DMB();        /* 先写数据再改 head */
    q->head = next;

    canq_exit_critical(pm);

	if (was_empty)
    {
	    q->last_progress_ms = HAL_GetTick();
    }

    can_tx_kick(q);
    return 0;
}

uint8_t can_send_std_ex(uint16_t std_id, const uint8_t *data, uint8_t len, CAN_TxPolicy_t policy)
{
    if (!data || len == 0 || len > 8) return 1;

    CAN_TxHeaderTypeDef h = {0};
    h.StdId = std_id;
    h.IDE   = CAN_ID_STD;
    h.RTR   = CAN_RTR_DATA;
    h.DLC   = len;
    h.TransmitGlobalTime = DISABLE;

    uint8_t buf[8] = {0};
    memcpy(buf, data, len);

    return can_tx_enqueue_ex(&g_can_txq, &h, buf, policy);
}

uint8_t can_send_std(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    return can_send_std_ex(std_id, data, len, CAN_TXQ_DROP_NEW);
}

/*===================== RX FIFO（push/pop 都做并发保护） =====================*/
uint8_t can_rxfifo_push(CAN_RxFIFO_t *f, const CAN_RxHeaderTypeDef *h, const uint8_t *d8)
{
    if (!f || !h || !d8) return 0;

    uint8_t ok = 0;
    uint32_t pm = canq_enter_critical();
    if (f->count < CAN_RX_FIFO_SIZE)
    {
        f->buf[f->head].header = *h;
        memcpy(f->buf[f->head].data, d8, 8);
        f->head = next_rx(f->head);
        f->count++;
        ok = 1;
    }
    else
    {
        f->overflow_cnt++;
    }
    canq_exit_critical(pm);
    return ok;
}

uint8_t can_rxfifo_pop(CAN_RxFIFO_t *f, CAN_RxHeaderTypeDef *h, uint8_t *d8)
{
    if (!f || !h || !d8) return 0;

    uint8_t ok = 0;
    uint32_t pm = canq_enter_critical();
    if (f->count > 0U)
    {
        *h = f->buf[f->tail].header;
        memcpy(d8, f->buf[f->tail].data, 8);
        f->tail = next_rx(f->tail);
        f->count--;
        ok = 1;
    }
    canq_exit_critical(pm);
    return ok;
}

/*===================== ISR/回调里调用的“轻量函数” =====================*/
void can_on_tx_complete_isr(void)
{
	g_can_txq.last_isr_tx_ms = HAL_GetTick();
    /* ISR 里直接 kick 没问题（函数很短），也可改成通知 task */
    can_tx_kick(&g_can_txq);
}

void can_on_error_isr(void)
{
	g_can_txq.last_err = HAL_CAN_GetError(&hcan1);

    if (g_can_txq.last_err & HAL_CAN_ERROR_BOF)
    {
        g_can_txq.busoff_cnt++;
	    g_can_txq.busoff_pending = 1U; /* 只标记，恢复放到 service 里做 */
    }
}

/*===================== 建议周期调用：busoff 恢复 + 补 kick =====================*/
void can_queue_service(void)
{
	uint32_t now = HAL_GetTick();
	uint32_t err = HAL_CAN_GetError(&hcan1);

	if (err != 0U)
	{
		g_can_txq.last_err = err;
		if ((err & HAL_CAN_ERROR_BOF) && (g_can_txq.busoff_pending == 0U))
		{
			g_can_txq.busoff_cnt++;
			g_can_txq.busoff_pending = 1U;
		}
	}

	if (g_can_txq.busoff_pending)
	{
		g_can_txq.busoff_pending = 0U;
		can_hw_recover(&g_can_txq);
		return;
	}

	if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_ERROR)
	{
		g_can_txq.stall_recover_cnt++;
		can_hw_recover(&g_can_txq);
		return;
	}

	if (g_can_txq.kick_lock != 0U)
	{
		if ((now - g_can_txq.last_kick_ms) > CAN_KICK_LOCK_TIMEOUT_MS)
		{
			uint32_t pm = canq_enter_critical();
			if ((g_can_txq.kick_lock != 0U) &&
				(now - g_can_txq.last_kick_ms) > CAN_KICK_LOCK_TIMEOUT_MS)
			{
				g_can_txq.kick_lock = 0U;
				g_can_txq.kick_unlock_cnt++;
			}
			canq_exit_critical(pm);
		}
	}

	if (g_can_txq.head != g_can_txq.tail)
	{
		if (g_can_txq.last_progress_ms == 0U)
		{
			g_can_txq.last_progress_ms = now;
		}

		if ((now - g_can_txq.last_progress_ms) > CAN_TX_STALL_MS)
		{
			g_can_txq.stall_recover_cnt++;
			can_hw_recover(&g_can_txq);
			return;
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
		{
			can_tx_kick(&g_can_txq);
		}
	}
	else
	{
		g_can_txq.last_progress_ms = 0U;
	}
}


/********************************** Your CANopen APIs *************************/
/* SDO/NMT：建议可靠 -> DROP_NEW（满了就让上层重试/限频） */
void Control_Mode_SET(uint8_t CANopen_ID, uint8_t CANopen_mode)
{
    uint8_t data[5];
    data[0] = 0x2F;
    data[1] = 0x60;
    data[2] = 0x60;
    data[3] = 0x00;
    data[4] = CANopen_mode;
    (void)can_send_std_ex(0x600 + CANopen_ID, data, 5, CAN_TXQ_DROP_NEW);
}

void CANopen_NMT(uint8_t cmd, uint8_t CANopen_ID)
{
    uint8_t data[2];
    data[0] = cmd;
    data[1] = CANopen_ID;
    (void)can_send_std_ex(0x000, data, 2, CAN_TXQ_DROP_NEW);
}

void SDO_Write_Data1(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint8_t DATA)
{
    uint8_t data[5];
    data[0] = 0x2f;
    data[1] = Index & 0xFF;
    data[2] = (Index >> 8) & 0xFF;
    data[3] = SubIndex;
    data[4] = DATA;
    (void)can_send_std_ex(0x600 + CANopen_ID, data, 5, CAN_TXQ_DROP_NEW);
}

void SDO_Write_Data2(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint16_t DATA)
{
    uint8_t data[6];
    data[0] = 0x2b;
    data[1] = Index & 0xFF;
    data[2] = (Index >> 8) & 0xFF;
    data[3] = SubIndex;
    data[4] = (uint8_t)(DATA & 0xFF);
    data[5] = (uint8_t)((DATA >> 8) & 0xFF);
    (void)can_send_std_ex(0x600 + CANopen_ID, data, 6, CAN_TXQ_DROP_NEW);
}

void SDO_Write_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, int32_t DATA)
{
    uint8_t data[8];
    data[0] = 0x23;
    data[1] = Index & 0xFF;
    data[2] = (Index >> 8) & 0xFF;
    data[3] = SubIndex;
    data[4] = (uint8_t)(DATA & 0xFF);
    data[5] = (uint8_t)((DATA >> 8) & 0xFF);
    data[6] = (uint8_t)((DATA >> 16) & 0xFF);
    data[7] = (uint8_t)((DATA >> 24) & 0xFF);
    (void)can_send_std_ex(0x600 + CANopen_ID, data, 8, CAN_TXQ_DROP_NEW);
}

void SDO_Read_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex)
{
    uint8_t data[5];
    data[0] = 0x40;
    data[1] = Index & 0xFF;
    data[2] = (Index >> 8) & 0xFF;
    data[3] = SubIndex;
    data[4] = 0x00;
    (void)can_send_std_ex(0x600 + CANopen_ID, data, 5, CAN_TXQ_DROP_NEW);
}

void RPDO1_Write_Cmd_Data4(uint8_t CANopen_ID, int32_t DATA)
{
    uint8_t data[4];

    data[0] = (uint8_t)(DATA & 0xFF);
    data[1] = (uint8_t)(DATA >> 8) & 0xFF;
    data[2] = (uint8_t)(DATA >> 16) & 0xFF;
    data[3] = (uint8_t)(DATA >> 24) & 0xFF;

    (void)can_send_std_ex(0x200 + CANopen_ID, data, 4, CAN_TXQ_REPLACE_BY_ID);
}

void RPDO2_Write_Cmd_Data4(uint8_t CANopen_ID, int32_t DATA)
{
    uint8_t data[4];

    data[0] = (uint8_t)(DATA & 0xFF);
    data[1] = (uint8_t)(DATA >> 8) & 0xFF;
    data[2] = (uint8_t)(DATA >> 16) & 0xFF;
    data[3] = (uint8_t)(DATA >> 24) & 0xFF;

    (void)can_send_std_ex(0x300 + CANopen_ID, data, 4, CAN_TXQ_REPLACE_BY_ID);
}

void PDO_Write_Cmd_Data6(uint8_t CANopen_ID, uint16_t rpdo1, int32_t rpdo2)
{
    uint8_t data[6];

    data[0] = rpdo1 & 0xFF;
    data[1] = (rpdo1 >> 8) & 0xFF;
    data[2] = (uint8_t)(rpdo2 & 0xFF);
    data[3] = (uint8_t)(rpdo2 >> 8) & 0xFF;
    data[4] = (uint8_t)(rpdo2 >> 16) & 0xFF;
    data[5] = (uint8_t)(rpdo2 >> 24) & 0xFF;

    (void)can_send_std_ex(0x200 + CANopen_ID, data, 6, CAN_TXQ_REPLACE_BY_ID);
}

