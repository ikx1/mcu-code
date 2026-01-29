#include "bsp_driver_led.h"

typedef enum
{
    GPIO_MODE_INPUT = 0x00,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERATE,
    GPIO_MODE_ANALOG,
}MY_GPIO_MODE;

typedef enum{
    GPIO_OTYPE_PP = 0,
    GPIO_OTYPE_OD,
}MY_GPIO_OTYPE;

typedef enum{
    GPIO_OSPEED_LOW = 0,
    GPIO_OSPEED_MIDNUM,
    GPIO_OSPEED_HIGH,
    GPIO_OSPEED_VERY_HIGH,
}MY_GPIO_OSPEED;

typedef enum{
    GPIO_PUPD_NOPULL = 0,
    GPIO_PUPD_PULL_UP,
    GPIO_PUPD_PULL_DOWN,
    GPIO_PUPD_RESERVED,
}MY_GPIO_PUPD;

typedef struct{
    uint32_t pin;
    MY_GPIO_MODE mode;
    MY_GPIO_OTYPE otype;
    MY_GPIO_OSPEED ospeed;
    MY_GPIO_PUPD pupd;
}MY_GPIO_InitTypeDef;

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} MY_GPIO_PinState;


void my_gpio_init(MY_GPIO_TypeDef* gpiox, MY_GPIO_InitTypeDef* gpio_init)
{
    uint32_t temp = 0x00000000;
    
    for(uint8_t i = 0; i < 16; i++)
    {
        if((gpio_init->pin & (0x01 << i)) == 0)
        {
            continue;
        }
        
        temp = gpiox->MODER;
        temp &= ~(0x03 << (i * 2));
        temp |= (gpio_init->mode << (i * 2));
        gpiox->MODER = temp;

        temp = gpiox->OTYPER;
        temp &= ~(0x01 << i);
        temp |= (gpio_init->otype << i);
        gpiox->OTYPER = temp;

        temp = gpiox->OSPEEDER;
        temp &= ~(0x03 << (i * 2));
        temp |= (gpio_init->ospeed << (i * 2));
        gpiox->OSPEEDER = temp;

        temp = gpiox->PUPDR;
        temp &= ~(0x03 << (i * 2));
        temp |= (gpio_init->pupd << (i * 2));
        gpiox->PUPDR = temp;
    }
}

void my_gpio_write_pin(MY_GPIO_TypeDef* gpiox, uint16_t pin, uint8_t pin_state)
{
    if(pin_state != GPIO_PIN_RESET)
    {
        gpiox->BSRR = pin;
    }
    else
    {
        gpiox->BSRR = (uint32_t)pin << 16U;
    }
}

void gpio_toggle_pin(MY_GPIO_TypeDef* gpiox, uint16_t pin)
{
    uint32_t odr = gpiox->ODR;

    
    gpiox->BSRR = ((odr & pin) << 16U) | (~odr & pin);

}

void bsp_led_init(void)
{
    MY_RCC_GPIOC_CLK_ENABLE();

    my_gpio_write_pin(MY_GPIOC, MY_GPIO_PIN_13, GPIO_PIN_RESET);

    MY_GPIO_InitTypeDef led_InitStructure = {
        .mode = GPIO_MODE_OUTPUT,
        .otype = GPIO_OTYPE_PP,
        .ospeed = GPIO_OSPEED_LOW,
        .pupd = GPIO_PUPD_PULL_DOWN,
        .pin = MY_GPIO_PIN_13,
    };

    my_gpio_init(MY_GPIOC, &led_InitStructure);
}


void bsp_led_on(void)
{
    my_gpio_write_pin(MY_GPIOC, MY_GPIO_PIN_13, GPIO_PIN_SET);
}

void bsp_led_off(void)
{
    my_gpio_write_pin(MY_GPIOC, MY_GPIO_PIN_13, GPIO_PIN_RESET);
}

