#include "timer_driver.h"
#include "timer.h"
#include "common.h"

uint16_t  timer_cnt; //定时器中断里的计数值
void on_tick(void)
{
	timer_cnt++;
}
void timer_timing(timer_timing_t *tim)
{
	timer_driver_timing(tim->timer,tim->time,tim->on_tick);
	if(tim->start_immediately==true)  timer_hw_start(tim->timer);
}
void timer_start(uint32_t timer)
{
	timer_hw_start(timer);
}
void timer_pwm_config(timer_pwm_t *pwms,uint8_t len)
{
	while(len)
	{
		len--;
		timer_hw_pwm(pwms->pwm,&(pwms->pin_pwm),pwms->pwm_T,pwms->pwm_high_level_width,pwms->port_mode_pwm);
		pwms++;
	}
}
void timer_pwm_start(timer_pwm_t *pwms,uint8_t len)
{
	while(len)
		{
		 len--;
	     timer_hw_pwm_start(pwms->pwm);
	     pwms++;
		}
}
void timer_measure_config(timer_measure_t *measures,uint8_t len)
{
  	while(len)
  	{
  	 len--;
  	timer_hw_measure(measures->measure,&(measures->pin_measure),measures->mode,measures->port_mode_measure);
  	measures++;
  	}
}
void timer_measure_start(timer_measure_t *measures,uint8_t len)
{
  	while(len)
  	{
  	 len--;
  	 timer_hw_measure_start(measures->measure);
  	 measures++;
  	}
}
uint16_t timer_measure_read(timer_measure_t *measuress)
{
    return measure_driver_read(measuress->measure);
}

void timer_encoder_config(encoder_t *encoders,uint8_t len)
{
  while(len)
  {
	  len--;
	  timer_hw_encoder(encoders->encoder1,encoders->encoder2,&(encoders->pin1),&(encoders->pin2),encoders->mode);
	  encoders++;
  }

}
void timer_encoder_start(encoder_t *encoders,uint8_t len)
{
  while(len)
  {
	  len--;
	  timer_hw_encoder_start(encoders->encoder1);
	  encoders++;
  }

}
uint16_t timer_encoder_read(encoder_t *encoders)
{
    return encoder_driver_read(encoders->mode,encoders->encoder1);
}



