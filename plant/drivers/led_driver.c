#include "led_driver.h"
#include <stdint.h>

#include "../third_part/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
#define is_led(v)       (v & led_all)
#define LED_ALL_PORT    (__HAL_RCC_GPIOA_CLK_ENABLE()|__HAL_RCC_GPIOC_CLK_ENABLE())

typedef struct
{
    GPIO_TypeDef * port; //port address
    uint16_t  pin;
} led_hw;

led_hw * led_get_hw(led_hw_t led)
{
    static led_hw ret;
    switch(led)
    {
        case led_pow_red:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_5;//C5
            break;
        case led_pow_green:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_13;
            break;
        case led_gps_red:
            ret.port = GPIOA;
            ret.pin = GPIO_PIN_0;
            break;
        case led_gps_green:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_3;//C3
            break;
        case led_rf_red:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_2;
            break;
        case led_rf_green:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_1;
            break;
        case led_net_red:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_0;
            break;
        case led_net_green:
            ret.port = GPIOC;
            ret.pin = GPIO_PIN_4;
            break;
        default:
            break;
    }
    return &ret;
}

void led_hw_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    led_hw * led;
    uint16_t i;
    /* LED GPIO Periph clock enable */
    //RCC_AHBPeriphClockCmd(LED_ALL_PORT, ENABLE);
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    for(i = 0; i < 16; i++)
    {
        if(!is_led((1 << i))) continue;
        led = led_get_hw((led_hw_t)(1 << i));
        GPIO_InitStructure.Pin = led->pin;
        HAL_GPIO_Init(led->port, &GPIO_InitStructure);
    }
    led_off(led_all);
}

void led_on(led_hw_t leds)
{
    led_hw * led;
    uint16_t i;
    for(i = 0; i < 16; i++)
    {
        if(!is_led(leds & (1 << i))) continue;
        led = led_get_hw((led_hw_t)(1 << i));
        HAL_GPIO_WritePin(led->port, led->pin,GPIO_PIN_RESET);
    }
}

void led_off(led_hw_t leds)
{
    led_hw * led;
    uint16_t  i;
    for(i = 0; i < 16; i ++)
    {
        if(!is_led(leds & (1 << i))) continue;
        led = led_get_hw((led_hw_t)(1 << i));
        HAL_GPIO_WritePin(led->port, led->pin,GPIO_PIN_SET);
    }
}

void led_toggle(led_hw_t leds)
{
    led_hw * led;
    uint16_t  i;
    for(i = 0; i < 16; i ++)
    {
        if(!is_led(leds & (1 << i))) continue;
        led = led_get_hw((led_hw_t)(1 << i));
        led->port->ODR ^= led->pin;
    }
}
