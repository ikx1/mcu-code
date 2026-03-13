/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* IWDG init function */
void IWDG_Init_StdPeriph(void)
{
    /* 使能对寄存器写访问 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* 预分频 */
    IWDG_SetPrescaler(IWDG_Prescaler_32);

    /* 重载值：0~0x0FFF */
    IWDG_SetReload(999);

    /* 立即装载 */
    IWDG_ReloadCounter();
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
