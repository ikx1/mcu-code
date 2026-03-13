/**
 * @file can.__CHAR_UNSIGNED__
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "can.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
void Can_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  //打开CAN1所用GPIO口
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE); 
  
  //配置的IO是PA12，CAN的TXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;                
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  //配置的IO是PA11，CAN的RXD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             //上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Can1_Init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;	 //禁止时间触发通信模式
	CAN_InitStructure.CAN_ABOM = ENABLE;	//总线自动恢复功能		软件对CAN_MCR寄存器的INRQ位进行置1随后清0后,一旦硬件检测到128次11位连续的隐性位,就退出离线状态
	/*
		当 TEC 大于 255 时，达到总线关闭状态，该状态由 CAN_ESR 寄存器的 BOFF 位指示。在总线关闭状态下， bxCAN 不能再发送和接收消息。
        bxCAN 可以自动或者应软件请求而从总线关闭状态中恢复（恢复错误主动状态），具体取决于 CAN_MCR 寄存器的 ABOM 位。但在两种情况下， 
				bxCAN 都必须至少等待 CAN 标准中指定的恢复序列完成（在 CANRX 上监测到 128 次 11 个连续隐性位）。
		如果 ABOM 位置 1， bxCAN 将在进入总线关闭状态后自动启动恢复序列。
		如果 ABOM 位清零，则软件必须请求 bxCAN 先进入再退出初始化模式，从而启动恢复序列。
		注意： 在初始化模式下， bxCAN 不会监视 CANRX 信号，因此无法完成恢复序列。 要进行恢复，bxCAN 必须处于正常模式。
	*/
                                  
	CAN_InitStructure.CAN_AWUM = ENABLE; //睡眠模式 通过清除CAN_MCR寄存器的SLEEP位,由软件唤醒
	CAN_InitStructure.CAN_NART = DISABLE;//自动重发功能使能  	CAN报文是否只发1次,不管发送的结果如何(成功/出错或仲裁丢失)
	CAN_InitStructure.CAN_RFLM = DISABLE;//FIFO锁定功能				在接收到溢出时FIFO未被锁定,当接收到FIFO报文未被读出,下一个收到的报文会覆盖原有的报文
	/*
		FIFO锁定功能主要用于管理接收邮箱
		如果不使能FIFO锁定功能,当FIFO使能时,则在FIFO邮箱已满之后,后续的信息将会覆盖最后接收的信息
		如果使能FIFO锁定功能,则在FIFO邮箱已满之后,将丢弃最新的消息，保留最原始的三则消息
	*/
	
	CAN_InitStructure.CAN_TXFP = DISABLE;//发送FIFO使能    		发送的FIFO优先级由报文的标识符来决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	//工作模式的配置:可配置为正常模式,静默模式,回环模式,静默回环模式;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;                        //重新同步跳跃宽度为1个时间单位,范围[CAN_SJW_1tq,CAN_SJW_4tq]
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;                        //时间段1为2个时间单位,范围[CAN_BS1_1tq,CAN_BS1_16tq]
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;                        //时间段2为3个时间单位,范围[CAN_BS2_1tq,CAN_BS2_8tq]
	CAN_InitStructure.CAN_Prescaler = 6;  	//设定了一个时间单位的长度为12,范围[1,1024]
	//关于CAN总线比特率计算
	/*
		NominalBitTime = 1*Tq +tBS1 +  tBS2
		tBS1 = tq * (TS1[3:0] + 1);
		tBS2 = tq * (TS2[2:0] + 1)，
		Tq = (BRP[9:0] + 1) * TPCLK;
		TPCLK = APB 时钟的时间周期;
		BRP[9:0], TS1[3:0] 和 TS2[2:0] 在 CAN_BTR 寄存器中定义;
	//-------------------------------------------------------------
			CAN 波特率 = RCC_APB1Periph_CAN1 / Prescaler / (SJW + BS1 + BS2);
		
		SJW = synchronisation_jump_width 
		BS = bit_segment
		本例中，设置CAN波特率为500Kbps		
		CAN 波特率 = 72000000 / 6 / (1 + 3 + 2) / = 2 MBps	
	*/
	CAN_Init(CAN1,&CAN_InitStructure); 
	
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//过滤器使能
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;//设定了指向过滤器的FIFO0 

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//掩码模式,此处可配置为掩码模式和列表模式
	CAN_FilterInitStructure.CAN_FilterNumber = 0;//选择过滤器0
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//给出过滤器位宽为32位
	
	//对扩展数据帧进行过滤:(只接收扩展数据帧)
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0002;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;	
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_TME, ENABLE);        //使能CAN1中断
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	 	   //配置为CAN1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;          //先占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		           //从优先级为3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;	 	   //配置为CAN1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;          //先占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		           //从优先级为3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t can_init(void)
{
	//初始化CAN接口用到的GPIO口  
	Can_GPIO_Init();  
	//初始化CAN配置
	Can1_Init();
	
	return 0;
}
