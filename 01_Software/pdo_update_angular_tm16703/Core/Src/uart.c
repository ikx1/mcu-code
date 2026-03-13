/**
 * @file uart.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/********************************** Includes *********************************/
#include "uart.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/

/** 
  * @brief  USART1 GPIO 配置
  * @param  无
  * @retval 无
*/
static void uart_gpio_init(USART_TypeDef* USARTx)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	
	if(USART1 == USARTx)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if(USART2 == USARTx)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else if(USART3 == USARTx)
	{
		MODBUS_USART_GPIO_APBxClkCmd(MODBUS_USART_GPIO_CLK, ENABLE);
		MODBUS_USART_APBxClkCmd(MODBUS_USART_CLK, ENABLE);

		GPIO_InitStructure.GPIO_Pin = MODBUS_USART_TX_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(MODBUS_USART_TX_GPIO_PORT, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = MODBUS_USART_RX_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(MODBUS_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	}
	else if(UART4 == USARTx)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}
	else if(UART5 == USARTx)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOD, &GPIO_InitStructure);	
	}
}

void uart_nvic_config(USART_TypeDef* USARTx, uint8_t PreemptPriority, uint8_t SubPriority)
{
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	IRQn_Type IRQn;

	if (USARTx == USART1)      IRQn = USART1_IRQn;
    else if (USARTx == USART2) IRQn = USART2_IRQn;
    else if (USARTx == USART3) IRQn = USART3_IRQn;
    else if (USARTx == UART4)  IRQn = UART4_IRQn;
    else if (USARTx == UART5)  IRQn = UART5_IRQn;
    else return;

	NVIC_InitStructure.NVIC_IRQChannel      = IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void uart_config(USART_TypeDef* USARTx, uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure = {0};

	USART_InitStructure.USART_BaudRate      = BaudRate;
	USART_InitStructure.USART_WordLength    = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits      = USART_StopBits_1;
	USART_InitStructure.USART_Parity        = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode          = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTx, &USART_InitStructure); 
}

void uart1_init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	uart_gpio_init(USART1);

	uart_config(USART1, 115200);

	uart_nvic_config(USART1, 5, 0);

	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel    = DMA1_Channel4_IRQn;  /* UART1 DMA1Tx*/ 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel    = DMA1_Channel5_IRQn; /* UART1 DMA1Rx*/  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); 
	USART_Cmd(USART1, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE); /* 使能DMA收发 */
}

/**
 * @brief  uart1 dma发送通道配置
 * @param  
 * @retval 
 */
void uart1_dmatx_config(uint8_t *mem_addr, uint32_t mem_size)
{
	DMA_InitTypeDef DMA_InitStructure = {0};

	DMA_DeInit(DMA1_Channel4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr 		= (uint32_t)mem_addr; 
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralDST; 	/* 传输方向:内存->外设 */
	DMA_InitStructure.DMA_BufferSize 			= mem_size; 
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M 					= DMA_M2M_Disable; 
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  

	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE); 
	DMA_ClearFlag(DMA1_IT_TC4);	/* 清除发送完成标识 */
	DMA_Cmd(DMA1_Channel4, ENABLE); 
}

/**
 * @brief  uart1 dma接收通道配置
 * @param  
 * @retval 
 */
void uart1_dmarx_config(uint8_t *mem_addr, uint32_t mem_size)
{
	DMA_InitTypeDef DMA_InitStructure = {0};

	DMA_DeInit(DMA1_Channel5); 
	DMA_Cmd(DMA1_Channel5, DISABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr 		= (uint32_t)mem_addr; 
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralSRC; 	/* 传输方向:外设->内存 */
	DMA_InitStructure.DMA_BufferSize 			= mem_size; 
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_M2M 					= DMA_M2M_Disable; 
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 

	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC|DMA_IT_HT|DMA_IT_TE, ENABLE);/* 使能DMA半满、溢满、错误中断 */
	DMA_ClearFlag(DMA1_IT_TC5);
	DMA_ClearFlag(DMA1_IT_HT5);
	DMA_Cmd(DMA1_Channel5, ENABLE); 
}

/**
 * @brief  获取DMA接收buf剩余空间
 * @param  
 * @retval 
 */
uint16_t uart1_get_dmarx_buf_remain_size(void)
{
	return DMA_GetCurrDataCounter(DMA1_Channel5);	
}

/**
 * @brief  uart1循环发送
 * @param  
 * @retval 
 */
void uart1_poll_send(const uint8_t *buf, uint16_t size)
{	
    uint16_t i = 0;
    
    for (i=0; i<size; i++)
    {
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			USART_SendData(USART1, *(buf+i));
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
}

void uart2_init(void)
{
	uart_gpio_init(MODBUS_ULTRASONIC_USARTx);

	uart_config(MODBUS_ULTRASONIC_USARTx, MODBUS_ULTRASONIC_BAUDRATE);

	USART_Cmd(MODBUS_ULTRASONIC_USARTx, ENABLE);
}

void uart3_init(void)
{
	uart_gpio_init(MODBUS_USARTx);

	uart_config(MODBUS_USARTx, MODBUS_USART_BAUDRATE);

	uart_nvic_config(MODBUS_USARTx, 5, 0);

	USART_ITConfig(MODBUS_USARTx, USART_IT_RXNE, ENABLE);	
	USART_ITConfig(MODBUS_USARTx, USART_IT_IDLE, ENABLE);

	USART_Cmd(MODBUS_USARTx, ENABLE);	    
}

void Modbus_SendStr(uint8_t data)
{
	while(USART_GetFlagStatus(MODBUS_USARTx, USART_FLAG_TC) == RESET);
	GPIO_SetBits(GPIOE, GPIO_Pin_15);
	USART_SendData(MODBUS_USARTx, data);
	while(USART_GetFlagStatus(MODBUS_USARTx, USART_FLAG_TC) == RESET);
	GPIO_ResetBits(GPIOE,GPIO_Pin_15);  //将PE15置为低   一直在接收状态
}

void Modbus_SendBuf(uint8_t *str, uint8_t len)
{
	for(uint8_t i = 0; i < len; i ++)
		Modbus_SendStr(str[i]);
}

void modbus_ultrasonic_sendbuf(uint8_t *str, uint8_t len)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	
	for(uint8_t i = 0; i < len; i ++)
	{
	
		USART_SendData(MODBUS_ULTRASONIC_USARTx, str[i]);
		while(USART_GetFlagStatus(MODBUS_ULTRASONIC_USARTx, USART_FLAG_TC) == RESET)
		{
		}
		
	}
	
	GPIO_ResetBits(GPIOE,GPIO_Pin_14);
}

void uart4_init(void)
{
	uart_gpio_init(UART4);

	uart_config(UART4, 115200);

	uart_nvic_config(UART4, 5, 0);

	USART_ITConfig(UART4, USART_IT_RXNE,ENABLE) ; 		//使能接收中断
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

	USART_Cmd(UART4, ENABLE);
}

void uart4_send(u8 data)
{
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC)==0){}
	USART_SendData(UART4, data);
}

void  uart4_send_str(uint8_t *send_str, uint8_t num)
{
	for(uint8_t i=0; i < num; i++)
	{
		uart4_send(send_str[i]);
	}	
}

void uart5_init(void)
{
	uart_gpio_init(UART5);

	uart_config(UART5, 115200);

	uart_nvic_config(UART5, 6, 0);

	USART_ITConfig(UART5, USART_IT_RXNE,ENABLE); 

	USART_Cmd(UART5, ENABLE);	
}
