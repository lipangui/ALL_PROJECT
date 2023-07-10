#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H

#include "common.h"

void led_hw_init(void);
void led_on(led_hw_t leds);
void led_off(led_hw_t leds);
void led_toggle(led_hw_t leds);

#endif
