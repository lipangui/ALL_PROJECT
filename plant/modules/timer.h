#ifndef TIMER_H_
#define TIMER_H_
#include "stdbool.h"
#include <stdint.h>
#include "common.h"
extern uint16_t timer_cnt;
typedef struct
{
	channel_t pwm;
	pin_t pin_pwm;
	timer_afio_mode port_mode_pwm;
	uint32_t pwm_T;
	uint32_t pwm_high_level_width;
}timer_pwm_t;
typedef struct
{
	channel_t measure;
	pin_t pin_measure;
	measure_mode_t mode;
	timer_afio_mode port_mode_measure;
}timer_measure_t;
typedef struct
{
	channel_t encoder1;
	channel_t encoder2;
	pin_t pin1;
	pin_t pin2;
	encoder_mode_t mode;
}encoder_t;
typedef struct
{
	timers_t timer;
	uint32_t time; //in us
	bool   start_immediately;
	void (*on_tick)(void);
}timer_timing_t;

void timer_timing(timer_timing_t *tim);
void on_tick(void);
void timer_start(uint32_t timer);

void timer_pwm_config(timer_pwm_t *pwms, uint8_t len);
void timer_pwm_start(timer_pwm_t *pwms,uint8_t len);

void timer_measure_config(timer_measure_t * measures, uint8_t len);
void timer_measure_start(timer_measure_t *measures,uint8_t len);
uint16_t timer_measure_read(timer_measure_t *measuress);

void timer_encoder_config(encoder_t *encoders,uint8_t len);
void timer_encoder_start(encoder_t *encoders,uint8_t len);
uint16_t timer_encoder_read(encoder_t *encoders);


#endif /* TIMER_H_ */







