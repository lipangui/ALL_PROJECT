#ifndef __LED_H
#define __LED_H

#include "common.h"

typedef enum
{
    led_mode_off = 0,       //全灭
    led_mode_on = 1,        //全亮
    led_mode_normal = 2,    //绿灯慢闪
    led_mode_suboptimal = 3,//次佳,绿灯快闪
    led_mode_warning = 4,   //红灯慢闪
    led_mode_error = 5,     //红灯快闪
    led_mode_config = 6,    //红绿交替慢闪
    led_mode_init = 7,      //红绿交替慢闪
    led_mode_upgrade = 8,   //红绿交替快闪
    led_mode_dead = 9,      //红灯常亮
} led_mode_t;

typedef enum
{
    led_sys = 0,
    led_gps = 1,
    led_net = 2,
    led_rf = 3
} led_t;

#define is_led(led) (led == led_sys || led == led_gps || led == led_net || led == led_rf)

void led_init(void);
void led_loop(void);
void led_set(led_t led, led_mode_t mode);

#endif
