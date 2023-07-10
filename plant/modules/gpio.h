#ifndef __GPIO_H
#define __GPIO_H
#include "common.h"
#include <stdint.h>
typedef struct
{
	gpio_pin_t pins;//io脚
	gpio_mode_t mode;//io模式：推挽，输入，模拟，复用等
} gpiox_t;//逻辑io的结构体

/*
函数功能：逻辑io初始化
函数形参：逻辑io结构体指针（多个io），io数
 */
void gpio_config(gpiox_t *gpioes, uint8_t len);//len指全部引脚数

/*
函数功能：外部中断使能
函数 形参：io，模式（上升沿触发或者下降沿），回调函数指针
 */
extern void gpio_event_enable(gpio_pin_t port_pin, gpio_event_type_t event_type,void (*gpio_event_handler)(void));
extern void gpio_event_disable(gpio_pin_t port_pin);
void gpio_event_handler(void);
/*
函数功能：adc_dma初始化以及赋值
函数形参：使能adc，dma内存地址，内存大小
*/
extern void gpio_adc_set_dma_buffer(adcx_t adcx, uint16_t * buffer, uint32_t size);//size 必须是相应adc所使用的通道数的整倍数
/*
函数功能：adc初始化
函数形参：adc结构体，回调函数指针
*/
extern void gpio_adc_config(adc_t *adc, void (*adc_handler)());
extern void gpio_adc_start(adcx_t adcx);
extern void gpio_adc_stop(adcx_t adcx);
void adc_handler(void);
//使用顺序
//		gpio_adc_set_dma_buffer;
//		gpio_adc_config(&adc_t1);
//		gpio_adc_config(&adc_t2);
//		....
//		gpio_adc_start(adc)

extern uint16_t gpio_adc_once(adcx_t adcx,gpio_pin_t pin, uint8_t circle);//立马设置ADC，采完一次就关掉然后返回值

extern void gpio_write_bit(gpio_pin_t port_pin, uint8_t bit);//逻辑电平写bit
extern uint8_t gpio_read_bit(gpio_pin_t port_pin);//逻辑电平读bit

void gpio_write(gpio_value_def_t gpio_value_def, uint32_t value);//并行io写操作，并行io结构体（io口数组，位数模式），需要写的值
uint32_t gpio_read(gpio_value_def_t gpio_value_def);//并行io读操作


#endif
