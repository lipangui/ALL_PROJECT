
#ifndef TIMER_DRIVER_H_
#define TIMER_DRIVER_H_
#include "common.h"
#define timer_number  8      //定时器个数
#define f1Prescaler_apb2 71//分频值  100us
#define f1Prescaler_apb1 71

 void timer_driver_timing(timers_t tim,uint32_t time,void(*on_tick)(void));
 void timer_hw_start(timers_t tim);

 void timer_hw_pwm(channel_t pwm,pin_t *pin,uint32_t T,uint32_t pwm_high_level_width,timer_afio_mode port_mode_pwm);
 void timer_hw_pwm_start(channel_t pwm);

 void timer_hw_measure(channel_t measure,pin_t *pin,measure_mode_t mode,timer_afio_mode port_mode_measure);
 void timer_hw_measure_start(channel_t  measure);
 uint16_t measure_driver_read(channel_t  measure);

 void timer_hw_encoder(channel_t encoder1,channel_t encoder2,pin_t *pin1,pin_t *pin2,encoder_mode_t mode);
 void timer_hw_encoder_start(channel_t encoder1);
 uint16_t encoder_driver_read(uint32_t encoder_mode,channel_t  encoder1);

#endif /* TIMER_DRIVER_H_ */
