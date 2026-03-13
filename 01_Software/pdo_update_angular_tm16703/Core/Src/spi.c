/**
 * @file spi.c
 * @author 未农 (wn)
 * @brief 
 * @version 0.1
 * @date 2025-04-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/********************************** Includes *********************************/
#include "spi.h"


/********************************** Defines **********************************/


/********************************** Variables ********************************/


/********************************** Functions ********************************/
void spi_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void spi_init(void)
{
    SPI_InitTypeDef SPI_InitStructure = {0};

    spi_gpio_init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/**
	 * 模式1
	 * 当空闲态时，SCK处于低电平，数据采样是在第2个边沿，也就是SCK由低电平到高电平的跳变，
	 * 所以数据采样是在上升沿（准备数据），（发送数据）数据发送是在下降沿。	
	*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;//SPI 设置为单线双向发送
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//设置为主 SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//SPI 发送接收 8 位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//时钟极性0   时钟空闲IDLE为低电平 0
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//时钟相位1  		数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	//内部 NSS 信号有 SSI 位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//波特率预分频值为 8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输从 MSB 位开始,高位先行
	SPI_InitStructure.SPI_CRCPolynomial = 7;			// CRC 值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);
 
	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
}

void spi_spitx_config(uint8_t *buf, uint32_t buf_size)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(SPI1 -> DR);//目标地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buf;//要发送数据地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		//外设作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = buf_size;	//DMA 通道的 DMA 缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为 8 位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//数据宽度为 8 位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;			//工作在普通模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 	//DMA 通道 3 拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			//禁止 DMA 通道的内存到内存传输
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
}

/**
 * @Description  	WS2812 启动DMA传输
 * @Param     	  {void}
 * @Return    	  {void}
*/
void ws2812_Send_Data(uint16_t size)
{
	DMA_Cmd(DMA1_Channel3, DISABLE );
	DMA_ClearFlag(DMA1_FLAG_TC3);    
 	DMA_SetCurrDataCounter(DMA1_Channel3, size);
 	DMA_Cmd(DMA1_Channel3, ENABLE);
	
//	while (DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET);
//	DMA_ClearFlag(DMA1_FLAG_TC3);
}
