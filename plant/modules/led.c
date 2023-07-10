#include "led.h"
#include "led_driver.h"
#include "system.h"
#include <stdint.h>

#define BREATH_INTERVAL   1500
#define SLOW_BLINKING     1000
#define FAST_BLINKING    250
//#define N 4

typedef struct
{
    led_mode_t  mode;
    led_hw_t    red;
    led_hw_t    green;
    uint16_t         interval;
} led_instance;

static led_instance leds[4];

static uint16_t led_get_interval(led_mode_t mode);
static void led_interval(led_instance led);

void led_init(void)
{
    led_hw_init();

    leds[led_sys].mode  = led_mode_init;
    leds[led_sys].red   = led_pow_red;
    leds[led_sys].green = led_pow_green;
    leds[led_sys].interval  = SLOW_BLINKING;

    leds[led_gps].mode  = led_mode_init;
    leds[led_gps].red   = led_gps_red;
    leds[led_gps].green = led_gps_green;
    leds[led_gps].interval  = SLOW_BLINKING;

    leds[led_net].mode  = led_mode_init;
    leds[led_net].red   = led_net_red;
    leds[led_net].green = led_net_green;
    leds[led_net].interval  = SLOW_BLINKING;

    leds[led_rf].mode  = led_mode_init;
    leds[led_rf].red   = led_rf_red;
    leds[led_rf].green = led_rf_green;
    leds[led_rf].interval  = SLOW_BLINKING;

    /*
    led_on(led_all_red);
    system_delay(1000);
    led_toggle(led_all);
    system_delay(1000);
    */
    led_off(led_all);
}

void led_loop(void)
{
    uint8_t i = 0;
    for(i = 0; i < 4; i++)
    {
        led_interval(leds[i]);
    }
}

void led_set(led_t led, led_mode_t mode)
{
    if(is_led(led))
    {
        leds[led].mode = mode;
        leds[led].interval = led_get_interval(mode);
    }
}

static uint16_t led_get_interval(led_mode_t mode)
{
    uint16_t  interval = 0xFFFF;
    switch(mode)
    {
        case led_mode_normal:
        case led_mode_warning:
        case led_mode_init:
        case led_mode_config:
            interval = SLOW_BLINKING;
            break;
        case led_mode_suboptimal:
        case led_mode_error:
        case led_mode_upgrade:
            interval = FAST_BLINKING;
            break;
        default:
            break;
    }
    return interval;
}

static void led_interval(led_instance led)
{
    uint8_t step = system_time / led.interval;
  //  uint8_t flash_step;
    switch(led.mode)
    {
        case led_mode_off:
            led_off(led.red | led.green);
            break;

        case led_mode_on:
            led_on(led.red | led.green);
            break;

        case led_mode_normal:
        case led_mode_suboptimal:
            led_off(led.red);
            if(step & 0x01) led_on(led.green);
            else led_off(led.green);
            break;

        case led_mode_warning:
        case led_mode_error:
            led_off(led.green);
            if(step & 0x01) led_off(led.red);
            else led_on(led.red);
            break;

        case led_mode_config:
        case led_mode_init:
        case led_mode_upgrade:
            if(step & 0x01)
            {
                led_on(led.green);
                led_off(led.red);
            }
            else
                {
                led_off(led.green);
                led_on(led.red);
            }
            break;

        case led_mode_dead:
            led_off(led.green);
            led_on(led.red);
            break;
/*        case led_mode_xx1:
        	flash_step=led.interval/N;
        	//led_off(led.green|led.red);
        	if(step&0x01)
        	{
        		led_on(led.green);
        	}
        	else
        	{
        		led_off(led.green);
        	}
        	if((system_time/flash_step) & 0x01)
        	{
        		led_on(led.red);
        	}
        	else
        	{
        		led_off(led.red);
        	}*/

    }

}
