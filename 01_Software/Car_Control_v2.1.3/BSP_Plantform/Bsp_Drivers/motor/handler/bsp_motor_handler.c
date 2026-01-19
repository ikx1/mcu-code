/**
 * @file bsp_can_handler.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "bsp_motor_handler.h"
#include "can_queue.h"

/********************************** Defines **********************************/


/********************************** Variables ********************************/
motor_info_t motor_infos[MOTOR_NUM] = {
	{.id = Right_Wheel_ID, .state = MOTOR_STATE_UNKNOWN},
    {.id = Left_Wheel_ID, .state = MOTOR_STATE_UNKNOWN},
};

static motor_driver_t motor_drivers[MOTOR_NUM];
extern CAN_RxFIFO_t can_rx_fifo;

/********************************** Functions ********************************/
motor_driver_t* get_motor_drivers_p(void)
{
	return &motor_drivers[0];
}

void motor_register(void)
{
    for (int i = 0; i < MOTOR_NUM; ++i)
        motor_driver_create(&motor_drivers[i], &motor_infos[i]);
}

//can 的接收函数
void ican_rec_analyse(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, uint8_t DLC)
{
	if ((RxHeader->RTR == CAN_RTR_DATA) && (RxHeader->IDE == CAN_ID_STD))
	{
		motor_driver_t *target_motor = NULL;

        // 匹配对应电机（这里只做两个为例）
        switch (RxHeader->StdId)
        {
            case 0x580 + Right_Wheel_ID: target_motor = &motor_drivers[0]; break;  
            case 0x580 + Left_Wheel_ID: target_motor = &motor_drivers[1]; break;  
            case 0x180 + Right_Wheel_ID: target_motor = &motor_drivers[0]; break;  
            case 0x180 + Left_Wheel_ID: target_motor = &motor_drivers[1]; break;  
            // case 0x280 + Right_Wheel_ID: target_motor = &motor_drivers[0]; break;  
            // case 0x280 + Left_Wheel_ID: target_motor = &motor_drivers[1]; break;  

			default: return;
        }

		if (!target_motor) return;

		if(DLC == 8)
		{
			uint16_t index = RxData[1] | (RxData[2] << 8);
			uint8_t cmd = RxData[0];
			switch(cmd)
			{
				case 0x43:	//回复4字节
					if(index == 0x6069)
					{
						int32_t speed = (int32_t)(
													RxData[4] |
											(RxData[5] << 8) |
											(RxData[6] << 16) |
											(RxData[7] << 24));
						
						target_motor->update_speed(target_motor, speed);
					}
					else if (index == 0x6063)  // 实际位置
					{
						int32_t pos = (int32_t)(
													RxData[4] |
											(RxData[5] << 8) |
											(RxData[6] << 16) |
											(RxData[7] << 24));

						target_motor->update_encoder(target_motor, pos);
					}
					break;
				
				case 0x4b:		//回复2字节
					if(index == 0x6041)
					{
						uint16_t status = (RxData[4] | (RxData[5] << 8));
						target_motor->update_state(target_motor, status);
					}
					break;

				default:
					break;				
			}	
		}
		else if(DLC == 6)
		{
			uint16_t status = (RxData[0] | (RxData[1] << 8));
			int32_t pos = (int32_t)(
													RxData[2] |
											(RxData[3] << 8) |
											(RxData[4] << 16) |
											(RxData[5] << 24));
			target_motor->update_state(target_motor, status);								
			target_motor->update_encoder(target_motor, pos);
			// target_motor->update_speed(target_motor, pos);
		}
		else return ;
	}

}

void can_poll_parse_loop(void)
{
	CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    while (can_rxfifo_pop(&g_can_rxfifo, &header, data))    
	{
        ican_rec_analyse(&header, data, header.DLC);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    CAN_RxHeaderTypeDef rh;
    uint8_t data[8];
	uint32_t pending = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);

	while (pending-- > 0U)
    {
	    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rh, data) != HAL_OK)
        {
            break;
        }
	    (void)can_rxfifo_push(&g_can_rxfifo, &rh, data);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}

/* 中止也要 kick，避免悬住 */
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_tx_complete_isr();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) can_on_error_isr();
}
