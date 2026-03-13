#include "tim.h"

void WS2811_TIM3_CH2_PWM_INIT(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    DMA_InitTypeDef dma;

    /* 开启时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* GPIOA7 (TIM3_CH2) */
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* TIM3 配置 */
    tim.TIM_Prescaler = 0;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = TIM3_ARR_VALUE;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tim);

    /* 通道2 PWM 模式 */
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		
    TIM_OC2Init(TIM3, &oc);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); // DMA 写直接更新

    /* DMA */
    dma.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR2;
    dma.DMA_MemoryBaseAddr = (uint32_t)ws2811_buf;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = WS2811_BUF_LEN;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &dma);
		
    TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM3, DISABLE);
}
