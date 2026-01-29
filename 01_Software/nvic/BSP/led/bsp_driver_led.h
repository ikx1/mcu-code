#ifndef __BSP_DRIVER_LED_H__
#define __BSP_DRIVER_LED_H__

#include <stdio.h>
#include <stdint.h>

#define MY_GPIO_PIN_0                ((uint16_t)0x0001)  /* Pin 0 selected    */
#define MY_GPIO_PIN_1                ((uint16_t)0x0002)  /* Pin 1 selected    */
#define MY_GPIO_PIN_2                ((uint16_t)0x0004)  /* Pin 2 selected    */
#define MY_GPIO_PIN_3                ((uint16_t)0x0008)  /* Pin 3 selected    */
#define MY_GPIO_PIN_4                ((uint16_t)0x0010)  /* Pin 4 selected    */
#define MY_GPIO_PIN_5                ((uint16_t)0x0020)  /* Pin 5 selected    */
#define MY_GPIO_PIN_6                ((uint16_t)0x0040)  /* Pin 6 selected    */    
#define MY_GPIO_PIN_7                ((uint16_t)0x0080)  /* Pin 7 selected    */
#define MY_GPIO_PIN_8                ((uint16_t)0x0100)  /* Pin 8 selected    */
#define MY_GPIO_PIN_9                ((uint16_t)0x0200)  /* Pin 9 selected    */
#define MY_GPIO_PIN_10               ((uint16_t)0x0400)  /* Pin 10 selected   */
#define MY_GPIO_PIN_11               ((uint16_t)0x0800)  /* Pin 11 selected   */
#define MY_GPIO_PIN_12               ((uint16_t)0x1000)  /* Pin 12 selected   */
#define MY_GPIO_PIN_13               ((uint16_t)0x2000)  /* Pin 13 selected   */
#define MY_GPIO_PIN_14               ((uint16_t)0x4000)  /* Pin 14 selected   */
#define MY_GPIO_PIN_15               ((uint16_t)0x8000)  /* Pin 15 selected   */


#define FLASH_BASE_ADDR     0x08000000UL

#define PERIPHER_BASE_ADDR  0x40000000UL

#define APB1_BASE_ADDR      PERIPHER_BASE_ADDR
#define APB2_BASE_ADDR      (APB1_BASE_ADDR + 0x00010000UL)
#define AHB1_BASE_ADDR      (APB2_BASE_ADDR + 0x00010000UL)
#define AHB2_BASE_ADDR      (APB1_BASE_ADDR + 0x10000000UL)

#define GPIOA_BASE_ADDR     (AHB1_BASE_ADDR + 0 * 0x400)
#define GPIOB_BASE_ADDR     (AHB1_BASE_ADDR + 1 * 0x400)
#define GPIOC_BASE_ADDR     (AHB1_BASE_ADDR + 2 * 0x400)
#define GPIOD_BASE_ADDR     (AHB1_BASE_ADDR + 3 * 0x400)
#define GPIOE_BASE_ADDR     (AHB1_BASE_ADDR + 4 * 0x400)
#define GPIOF_BASE_ADDR     (AHB1_BASE_ADDR + 5 * 0x400)
#define GPIOG_BASE_ADDR     (AHB1_BASE_ADDR + 6 * 0x400)
#define GPIOH_BASE_ADDR     (AHB1_BASE_ADDR + 7 * 0x400)
#define GPIOI_BASE_ADDR     (AHB1_BASE_ADDR + 8 * 0x400)

#define MY_RCC_BASE_ADDR    (0x40023800UL)

#define MY_RCC_GPIOC_CLK_ENABLE()  (*((volatile uint32_t *) (MY_RCC_BASE_ADDR + 0x30)) |= (1U << 2))

#define MY_GPIOA   ((MY_GPIO_TypeDef *) GPIOA_BASE_ADDR)
#define MY_GPIOB   ((MY_GPIO_TypeDef *) GPIOB_BASE_ADDR)
#define MY_GPIOC   ((MY_GPIO_TypeDef *) GPIOC_BASE_ADDR)
#define MY_GPIOD   ((MY_GPIO_TypeDef *) GPIOD_BASE_ADDR)
#define MY_GPIOE   ((MY_GPIO_TypeDef *) GPIOE_BASE_ADDR)
#define MY_GPIOF   ((MY_GPIO_TypeDef *) GPIOF_BASE_ADDR)
#define MY_GPIOG   ((MY_GPIO_TypeDef *) GPIOG_BASE_ADDR)
#define MY_GPIOH   ((MY_GPIO_TypeDef *) GPIOH_BASE_ADDR)
#define MY_GPIOI   ((MY_GPIO_TypeDef *) GPIOI_BASE_ADDR)

typedef struct
{
    volatile uint32_t MODER;    //GPIO Port moder register
    volatile uint32_t OTYPER;   //GPIO Port output type register
    volatile uint32_t OSPEEDER; //GPIO Port output speed register
    volatile uint32_t PUPDR;    //GPIO Port pull-up/pull-down register
    volatile uint32_t IDR;//GPIO Port input data register
    volatile uint32_t ODR;//GPIO Port output data register
    volatile uint32_t BSRR;//GPIO Port bit set/reset register
    volatile uint32_t LCKR;//GPIO Port configuration lock register
    volatile uint32_t AFRL;//GPIO alternate function low register
    volatile uint32_t AFRH;//GPIO alternate function high register
} MY_GPIO_TypeDef;

void bsp_led_init(void);
void bsp_led_on(void);
void bsp_led_off(void);
void gpio_toggle_pin(MY_GPIO_TypeDef* gpiox, uint16_t pin);

#endif /* __BSP_DRIVER_LED_H__ */
