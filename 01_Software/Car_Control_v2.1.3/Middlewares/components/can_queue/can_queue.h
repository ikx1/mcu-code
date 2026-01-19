#ifndef __CAN_QUEUE_H__
#define __CAN_QUEUE_H__

#include <stdint.h>
#include <string.h>
#include "can.h"   /* hcan1, CAN_xxx types */

/*===================== 可配置参数（必须 2^n） =====================*/
#ifndef CAN_QUEUE_NUM
#define CAN_QUEUE_NUM        128U
#endif

#ifndef CAN_RX_FIFO_SIZE
#define CAN_RX_FIFO_SIZE     128U
#endif

/*===================== runtime watchdogs =====================*/
#ifndef CAN_TX_STALL_MS
#define CAN_TX_STALL_MS      500U
#endif

#ifndef CAN_KICK_LOCK_TIMEOUT_MS
#define CAN_KICK_LOCK_TIMEOUT_MS  50U
#endif

/*===================== 发送策略 =====================*/
typedef enum
{
    CAN_TXQ_DROP_NEW = 0,     /* 队列满：丢新帧（SDO/NMT等可靠命令，满了让上层限频/重试） */
    CAN_TXQ_DROP_OLD,         /* 队列满：丢最老帧（实时类帧） */
    CAN_TXQ_REPLACE_BY_ID     /* 若队列中存在相同 StdId(含IDE/RTR/DLC) 的“未入邮箱帧”：直接覆盖 */
} CAN_TxPolicy_t;

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CAN_TxItem_t;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint8_t  kick_lock;

    CAN_TxItem_t buf[CAN_QUEUE_NUM];

    /* 统计 */
    volatile uint32_t drop_cnt;
    volatile uint32_t replace_cnt;
    volatile uint32_t addtx_fail_cnt;
    volatile uint32_t kick_cnt;
    volatile uint32_t busoff_cnt;
    volatile uint32_t recover_cnt;
    volatile uint32_t stall_recover_cnt;
    volatile uint32_t kick_unlock_cnt;

    volatile uint32_t last_err;        /* HAL_CAN_GetError() */
    volatile uint8_t  busoff_pending;  /* ISR 标记，task/service 中恢复 */
    volatile uint32_t last_kick_ms;
    volatile uint32_t last_progress_ms;
    volatile uint32_t last_isr_tx_ms;
} CANBUS_QUEUE_INFO;

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_RxItem_t;

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    CAN_RxItem_t buf[CAN_RX_FIFO_SIZE];
    volatile uint32_t overflow_cnt;
} CAN_RxFIFO_t;

extern CANBUS_QUEUE_INFO g_can_txq;
extern CAN_RxFIFO_t      g_can_rxfifo;

/*===================== 基础接口 =====================*/
void     can_queue_init(void);              /* 配置 filter + start + notify + 清队列 */
void     can_queue_service(void);           /* 建议 10~50ms 调一次：busoff 恢复/补 kick */
void     can_tx_kick(CANBUS_QUEUE_INFO *q); /* 填满邮箱：tail 在“成功塞入邮箱”时前移 */

uint8_t  can_tx_enqueue_ex(CANBUS_QUEUE_INFO *q,
                           const CAN_TxHeaderTypeDef *h,
                           const uint8_t *data8,
                           CAN_TxPolicy_t policy);

uint8_t  can_send_std_ex(uint16_t std_id, const uint8_t *data, uint8_t len, CAN_TxPolicy_t policy);
uint8_t  can_send_std(uint16_t std_id, const uint8_t *data, uint8_t len);

/*===================== RX FIFO =====================*/
uint8_t  can_rxfifo_push(CAN_RxFIFO_t *f, const CAN_RxHeaderTypeDef *h, const uint8_t *d8);
uint8_t  can_rxfifo_pop (CAN_RxFIFO_t *f, CAN_RxHeaderTypeDef *h, uint8_t *d8);

/*===================== HAL Hook（放你工程里即可） =====================*/
void can_filter_init_all_pass(void);

/* 在回调里调用：建议直接用下面的默认实现 */
void can_on_tx_complete_isr(void);
void can_on_error_isr(void);
						   						   
void Control_Mode_SET(uint8_t CANopen_ID, uint8_t CANopen_mode);
void CANopen_NMT(uint8_t cmd, uint8_t CANopen_ID);

void SDO_Write_Data1(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint8_t DATA);
void SDO_Write_Data2(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, uint16_t DATA);
void SDO_Write_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex, int32_t DATA);
void SDO_Read_Data4(uint8_t CANopen_ID, uint16_t Index, uint8_t SubIndex);

void RPDO1_Write_Cmd_Data4(uint8_t CANopen_ID, int32_t DATA);
void RPDO2_Write_Cmd_Data4(uint8_t CANopen_ID, int32_t DATA);
void PDO_Write_Cmd_Data6(uint8_t CANopen_ID, uint16_t rpdo1, int32_t rpdo2);

#endif /* __CAN_QUEUE_H__ */
