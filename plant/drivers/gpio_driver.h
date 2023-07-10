/*
 * gpio_driver.h
 *
 *  Created on: 2020年3月25日
 *      Author: user
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_
#include "common.h"
#include <stdint.h>

#define GPIO_PIN_NUM 16
void adc_init(void);
void gpio_hw_config(gpio_mode_t mode,gpio_pin_t pins);

void gpio_event_enable(gpio_pin_t port_pin, gpio_event_type_t event_type,void (*gpio_event_handler)(void));
void gpio_event_disable(gpio_pin_t port_pin);

void gpio_adc_set_dma_buffer(adcx_t adcx, uint16_t * buffer, uint32_t size);
void gpio_adc_config(adc_t *adc_t, void (*adc_handler)(void));
void gpio_adc_start(adcx_t adcx);
void gpio_adc_stop(adcx_t adcx);
//uint16_t adc_driver_read(gpio_pin_t pin, uint16_t * value);
uint16_t gpio_adc_once(adcx_t adcx,gpio_pin_t pin, uint8_t circle);

void gpio_write_bit(gpio_pin_t port_pin, uint8_t bit);
uint8_t gpio_read_bit(gpio_pin_t port_pin);

void gpio_driver_write(gpio_value_def_t gpio_value_def, uint32_t value);
uint32_t gpio_driver_read(gpio_value_def_t gpio_value_def);

#endif /* GPIO_DRIVER_H_ */
