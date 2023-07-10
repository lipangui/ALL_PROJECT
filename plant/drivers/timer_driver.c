#include "timer_driver.h"
#include "common.h"

#include "../third_part/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"

typedef struct
{
	TIM_TypeDef *timer;
	IRQn_Type IRQn;
	GPIO_TypeDef *gpio;
}timer_tim;

timer_tim tims[timer_number];

TIM_HandleTypeDef TIM[timer_number];

TIM_IC_InitTypeDef TIM_MEASURE[13];
void (*timer_on_tick)(void);

uint16_t timer_cnt=0;
uint16_t capure_buf[4][3];
uint8_t capure_cnt=0;
uint16_t pulse_width[4];

uint16_t endcoder_speed=0;
uint16_t endcoder_position=0;

static void timer_TIM_init(timer_tim *tims)
{
	tims[timer1].timer = TIM1;
	tims[timer2].timer = TIM2;
	tims[timer3].timer = TIM3;
	tims[timer4].timer = TIM4;
/*	tims[timer5].timer = TIM5;
	tims[timer6].timer = TIM6;
	tims[timer7].timer = TIM7;
	tims[timer8].timer = TIM8;*/
	tims[timer1].IRQn = TIM1_UP_IRQn;
	tims[timer2].IRQn = TIM2_IRQn;
	tims[timer3].IRQn = TIM3_IRQn;
	tims[timer4].IRQn = TIM4_IRQn;
/*  tims[timer5].IRQn = TIM5_IRQn;
	tims[timer6].IRQn = TIM6_IRQn;
	tims[timer7].IRQn = TIM7_IRQn;
	tims[timer8].IRQn = TIM8_UP_IRQn;*/

}
static void timer_rcc_init(timers_t tim)
{
switch(tim)
	{
	case timer1:
		    __HAL_RCC_TIM1_CLK_ENABLE();
		    TIM[timer1].Init.Prescaler=f1Prescaler_apb2;
		break;
	case timer2:
			__HAL_RCC_TIM2_CLK_ENABLE();
		    TIM[timer2].Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer3:
			__HAL_RCC_TIM3_CLK_ENABLE();
			 TIM[timer3].Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer4:
			__HAL_RCC_TIM4_CLK_ENABLE();
			 TIM[timer4].Init.Prescaler=f1Prescaler_apb1;
			break;
/*	case timer5:
			__HAL_RCC_TIM5_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer6:
			__HAL_RCC_TIM6_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer7:
			__HAL_RCC_TIM7_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer8:
			__HAL_RCC_TIM8_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb2;
			break;*/
	default:
			break;
	}
}

static void timer_AFIO_init(timers_t tim,timer_afio_mode mode)
{
switch(tim)
	{
	case timer1:
		if(mode==afio_part)   __HAL_AFIO_REMAP_TIM1_PARTIAL();
		if(mode==afio_all)    __HAL_AFIO_REMAP_TIM1_ENABLE();
		break;
	case timer2:
		if(mode==afio_part)   __HAL_AFIO_REMAP_TIM2_PARTIAL_1();__HAL_AFIO_REMAP_TIM2_PARTIAL_2();
		if(mode==afio_all)    __HAL_AFIO_REMAP_TIM2_ENABLE();
			break;
	case timer3:
		if(mode==afio_part)   __HAL_AFIO_REMAP_TIM3_PARTIAL();
		if(mode==afio_all)    __HAL_AFIO_REMAP_TIM3_ENABLE();
			break;
	case timer4:
		//if(mode==afio_part)   __HAL_AFIO_REMAP_TIM4_PARTIAL();
		if(mode==afio_all)    __HAL_AFIO_REMAP_TIM4_ENABLE();
			break;
/*	case timer5:
			__HAL_RCC_TIM5_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer6:
			__HAL_RCC_TIM6_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer7:
			__HAL_RCC_TIM7_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb1;
			break;
	case timer8:
			__HAL_RCC_TIM8_CLK_ENABLE();
			 TIM.Init.Prescaler=f1Prescaler_apb2;
			break;*/
	default:
			break;
	}
          __HAL_AFIO_REMAP_SWJ_NOJTAG();
}
static void timer_gpio_init(pin_t *pin,GPIO_InitTypeDef  *GPIO_InitStruct)
{
switch(pin->port)
{
    case 1:__HAL_RCC_GPIOA_CLK_ENABLE(); tims->gpio=GPIOA;
	       break;
	case 2:__HAL_RCC_GPIOB_CLK_ENABLE(); tims->gpio=GPIOB;
	       break;
	case 3:__HAL_RCC_GPIOC_CLK_ENABLE(); tims->gpio=GPIOC;
	       break;
	case 4:__HAL_RCC_GPIOD_CLK_ENABLE(); tims->gpio=GPIOD;
	       break;
	default:
			break;
}
switch(pin->pins)
	{
	case (0x1<<0) : GPIO_InitStruct->Pin = GPIO_PIN_0; break;
	case (0x1<<1) : GPIO_InitStruct->Pin = GPIO_PIN_1; break;
	case (0x1<<2) : GPIO_InitStruct->Pin = GPIO_PIN_2; break;
	case (0x1<<3) : GPIO_InitStruct->Pin = GPIO_PIN_3; break;
	case (0x1<<4) : GPIO_InitStruct->Pin = GPIO_PIN_4; break;
	case (0x1<<5) : GPIO_InitStruct->Pin = GPIO_PIN_5; break;
	case (0x1<<6) : GPIO_InitStruct->Pin = GPIO_PIN_6; break;
	case (0x1<<7) : GPIO_InitStruct->Pin = GPIO_PIN_7; break;
	case (0x1<<8) : GPIO_InitStruct->Pin = GPIO_PIN_8; break;
	case (0x1<<9) : GPIO_InitStruct->Pin = GPIO_PIN_9; break;
	case (0x1<<10): GPIO_InitStruct->Pin = GPIO_PIN_10;break;
	case (0x1<<11): GPIO_InitStruct->Pin = GPIO_PIN_11;break;
	case (0x1<<12): GPIO_InitStruct->Pin = GPIO_PIN_12;break;
	case (0x1<<13): GPIO_InitStruct->Pin = GPIO_PIN_13;break;
	case (0x1<<14): GPIO_InitStruct->Pin = GPIO_PIN_14;break;
	case (0x1<<15): GPIO_InitStruct->Pin = GPIO_PIN_15;break;
	default:break;
	}
}
void timer_driver_timing(timers_t tim,uint32_t time,void(*on_tick)(void))
{
	  timer_TIM_init(tims);
	  timer_rcc_init(tim);
	  timer_on_tick=on_tick;
	  TIM[tim].Instance=tims[tim].timer;
      TIM[tim].Init.Period=time;
	  TIM[tim].Init.CounterMode=TIM_COUNTERMODE_UP;
	  TIM[tim].Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	  TIM[tim].Init. RepetitionCounter=0;
	  HAL_TIM_Base_Init(&TIM[tim]);

	  HAL_NVIC_SetPriority(tims[tim].IRQn,tim/4,tim%4);
	  HAL_NVIC_EnableIRQ(tims[tim].IRQn);
}
void timer_hw_start(timers_t tim)
{
  HAL_TIM_Base_Start_IT(&TIM[tim]);
}
uint8_t timer_select(channel_t channel)
{
if(channel&(0xf<<0))       return timer1;
if(channel&(0xf<<4))       return timer2;
if(channel&(0xf<<8))       return timer3;
if(channel&(0xf<<12))      return timer4;
/*
if(channel&(0xf<<16))     return timer5;
if(channel&(0xf<<20))     return timer8;
*/
return -1;
}

uint32_t channel_select(channel_t channel)
{
 channel_t channels;
  if(channel&(0xf<<0))  channels=channel;
  if(channel&(0xf<<4))  channels=channel>>4;
  if(channel&(0xf<<8))  channels=channel>>8;
  if(channel&(0xf<<12)) channels=channel>>12;
 switch(channels%8)
 {
 case 0:return TIM_CHANNEL_4;
        break;
 case 1:return TIM_CHANNEL_1;
         break;
 case 2:return TIM_CHANNEL_2;
         break;
 case 4:return TIM_CHANNEL_3;
         break;
 default:return -1;
 		 break;
 }
}
void timer_hw_pwm(channel_t pwm,pin_t *pin,uint32_t T,uint32_t pwm_high_level_width,timer_afio_mode port_mode_pwm)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  TIM_ClockConfigTypeDef TIM_PWM_CLOCK;
	  TIM_OC_InitTypeDef TIM_PWM;
	  timer_TIM_init(tims);
	  timer_rcc_init(timer_select(pwm));

	  TIM[timer_select(pwm)].Instance =tims[timer_select(pwm)].timer;
	  TIM[timer_select(pwm)].Init.CounterMode = TIM_COUNTERMODE_UP;
	  TIM[timer_select(pwm)].Init.Period = T;
	  TIM[timer_select(pwm)].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  TIM[timer_select(pwm)].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_PWM_Init(&TIM[timer_select(pwm)]);

	  TIM_PWM_CLOCK.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(&TIM[timer_select(pwm)], &TIM_PWM_CLOCK);

	  TIM_PWM.OCMode = TIM_OCMODE_PWM1;
	  TIM_PWM.Pulse = (pwm_high_level_width*T)/100;
	  TIM_PWM.OCPolarity = TIM_OCPOLARITY_HIGH;
	  TIM_PWM.OCFastMode = TIM_OCFAST_DISABLE;
	  HAL_TIM_PWM_ConfigChannel(&TIM[timer_select(pwm)], &TIM_PWM, channel_select(pwm));

	  GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	  timer_gpio_init(pin,&GPIO_InitStruct);
	  HAL_GPIO_Init(tims->gpio, &GPIO_InitStruct);

	  timer_AFIO_init(timer_select(pwm),port_mode_pwm);
}

void timer_hw_pwm_start(channel_t pwm)
{
	HAL_TIM_PWM_Start(&TIM[timer_select(pwm)], channel_select(pwm));
}
void timer_hw_measure(channel_t measure,pin_t *pin,measure_mode_t mode,timer_afio_mode port_mode_measure)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  TIM_ClockConfigTypeDef TIM_MEASURE_CLOCK ;
	  timer_TIM_init(tims);
	  timer_rcc_init(timer_select(measure));

	  TIM[timer_select(measure)].Instance = tims[timer_select(measure)].timer;
	  TIM[timer_select(measure)].Init.CounterMode = TIM_COUNTERMODE_UP;
	  TIM[timer_select(measure)].Init.Period = 0xffff;
	  TIM[timer_select(measure)].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  TIM[timer_select(measure)].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_IC_Init(&TIM[timer_select(measure)]);

	  TIM_MEASURE_CLOCK.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(&TIM[timer_select(measure)], &TIM_MEASURE_CLOCK);

     if(mode == hight)
	  {
    	 TIM_MEASURE[channel_select(measure)].ICPolarity  = TIM_ICPOLARITY_RISING;
	  }
     else if(mode == low)
     {
    	 TIM_MEASURE[channel_select(measure)].ICPolarity  = TIM_ICPOLARITY_FALLING;
     }
	  TIM_MEASURE[channel_select(measure)].ICSelection = TIM_ICSELECTION_DIRECTTI;
	  TIM_MEASURE[channel_select(measure)].ICPrescaler = TIM_ICPSC_DIV1;
	  TIM_MEASURE[channel_select(measure)].ICFilter = 0;
	  HAL_TIM_IC_ConfigChannel(&TIM[timer_select(measure)], &TIM_MEASURE[channel_select(measure)], channel_select(measure));

	  GPIO_InitStruct.Mode=GPIO_MODE_AF_INPUT;
	  GPIO_InitStruct.Pull=GPIO_PULLDOWN;
	  timer_gpio_init(pin,&GPIO_InitStruct);
	  HAL_GPIO_Init(tims->gpio, &GPIO_InitStruct);

	  timer_AFIO_init(timer_select(measure),port_mode_measure);

	  HAL_NVIC_SetPriority(tims[timer_select(measure)].IRQn,0,timer_select(measure));
	  HAL_NVIC_EnableIRQ(tims[timer_select(measure)].IRQn);

}

void timer_hw_measure_start(channel_t  measure)
{
	HAL_TIM_IC_Start_IT(&TIM[timer_select(measure)], channel_select(measure));
}
uint16_t measure_driver_read(channel_t  measure)
{
	switch(channel_select(measure))
	{
	case TIM_CHANNEL_1:return pulse_width[0]; break;
	case TIM_CHANNEL_2:return pulse_width[1]; break;
	case TIM_CHANNEL_3:return pulse_width[2]; break;
	case TIM_CHANNEL_4:return pulse_width[3]; break;
	}
	return -1;
}
void timer_hw_encoder(channel_t encoder1,channel_t encoder2,pin_t *pin1,pin_t *pin2,encoder_mode_t mode)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  TIM_Encoder_InitTypeDef TIM_ENCODER;
	  timer_TIM_init(tims);
	  timer_rcc_init(timer_select(encoder1));

	  TIM[timer_select(encoder1)].Instance = tims[timer_select(encoder1)].timer;
	  TIM[timer_select(encoder1)].Init.CounterMode = TIM_COUNTERMODE_UP;
	  TIM[timer_select(encoder1)].Init.Prescaler=0;
	  TIM[timer_select(encoder1)].Init.Period = 65535;
	  TIM[timer_select(encoder1)].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  TIM[timer_select(encoder1)].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	  TIM_ENCODER.EncoderMode = TIM_ENCODERMODE_TI12;
	  TIM_ENCODER.IC1Polarity = TIM_ICPOLARITY_RISING;
	  TIM_ENCODER.IC1Filter = 0;
	  TIM_ENCODER.IC2Polarity = TIM_ICPOLARITY_RISING;
	  TIM_ENCODER.IC2Filter = 0;
	  HAL_TIM_Encoder_Init(&TIM[timer_select(encoder1)], &TIM_ENCODER);

	  GPIO_InitStruct.Mode=GPIO_MODE_AF_INPUT;
	  GPIO_InitStruct.Pull=GPIO_PULLDOWN;
	  timer_gpio_init(pin1,&GPIO_InitStruct);
	  HAL_GPIO_Init(tims->gpio, &GPIO_InitStruct);
	  timer_gpio_init(pin2,&GPIO_InitStruct);
	  HAL_GPIO_Init(tims->gpio, &GPIO_InitStruct);

	  __HAL_TIM_SET_COUNTER(&TIM[timer_select(encoder1)],32768);
}

void timer_hw_encoder_start(channel_t  encoder1)
{
	HAL_TIM_Encoder_Start(&TIM[timer_select(encoder1)],TIM_CHANNEL_ALL);
}

uint16_t encoder_driver_read(uint32_t encoder_mode,channel_t  encoder1)
{
 if( encoder_mode == speed_mode)
 {
	endcoder_speed=__HAL_TIM_GET_COUNTER(&TIM[timer_select(encoder1)])-32768;
	 __HAL_TIM_SET_COUNTER(&TIM[timer_select(encoder1)],32768);
	 return endcoder_speed;
 }
 else if(encoder_mode == position_mode)
 {
	 endcoder_position=__HAL_TIM_GET_COUNTER(&TIM[timer_select(encoder1)])-32768;
	 __HAL_TIM_SET_COUNTER(&TIM[timer_select(encoder1)],32768);
	 return endcoder_position;
 }
 return -1;
}

static void HAL_TIM_IC_Capturex(TIM_HandleTypeDef *htim,TIM_IC_InitTypeDef *TIM_MEASURE,uint32_t channel,uint16_t n)
{

	 switch(capure_cnt)
		{
		  case 0: capure_buf[n][0]=HAL_TIM_ReadCapturedValue(htim,channel);
		                        TIM_RESET_CAPTUREPOLARITY(htim,channel);
		                        TIM_SET_CAPTUREPOLARITY(htim,channel,~TIM_MEASURE->ICPolarity);
		                        capure_cnt++;
		          break;
		  case 1: capure_buf[n][1]=HAL_TIM_ReadCapturedValue(htim,channel);
		                        TIM_RESET_CAPTUREPOLARITY(htim,channel);
	                            TIM_SET_CAPTUREPOLARITY(htim,channel,TIM_MEASURE->ICPolarity);
		 	                    capure_cnt++;
		 	      break;
		  case 2: capure_buf[n][2]=HAL_TIM_ReadCapturedValue(htim,channel);
	                            TIM_RESET_CAPTUREPOLARITY(htim,channel);
	                            TIM_SET_CAPTUREPOLARITY(htim,channel,TIM_MEASURE->ICPolarity);
		  	 	                capure_cnt++;
		  	 	  break;
		  case 3: capure_cnt=0;pulse_width[n]=((capure_buf[n][1]-capure_buf[n][0]+1)*100/(capure_buf[n][2]-capure_buf[n][0]));

		         break;
		}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	switch(htim->Channel)
	{
        case HAL_TIM_ACTIVE_CHANNEL_1:
        	HAL_TIM_IC_Capturex(htim,&TIM_MEASURE[TIM_CHANNEL_1],TIM_CHANNEL_1,0);
        	break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
        	HAL_TIM_IC_Capturex(htim,&TIM_MEASURE[TIM_CHANNEL_2],TIM_CHANNEL_2,1);
        	break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
        	HAL_TIM_IC_Capturex(htim,&TIM_MEASURE[TIM_CHANNEL_3],TIM_CHANNEL_3,2);
        	break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
        	HAL_TIM_IC_Capturex(htim,&TIM_MEASURE[TIM_CHANNEL_4],TIM_CHANNEL_4,3);
        	break;
        default:break;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	switch(timer_cnt%4)
	{
	case 0:GPIOC->ODR |=(1<<13);
	       GPIOC->ODR &=~(1<<14);
	       break;
	case 1:GPIOC->ODR |=(1<<13);
	       GPIOC->ODR |=(1<<14);
	 	   break;
	case 2:GPIOC->ODR &=~(1<<13);
	       GPIOC->ODR |=(1<<14);
		   break;
	case 3:GPIOC->ODR &=~(1<<13);
	       GPIOC->ODR &=~(1<<14);
		   break;
	}
	timer_cnt++;
}
void MY_HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{

  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_1;


        if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)
        {
          HAL_TIM_IC_CaptureCallback(htim);
        }
        htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
      }
    }
  }

  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_2;

      if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)
      {
        HAL_TIM_IC_CaptureCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }

  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC3) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_3;

      if ((htim->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U)
      {
        HAL_TIM_IC_CaptureCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC4) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_4;

      if ((htim->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U)
      {
    	  HAL_TIM_IC_CaptureCallback(htim);
      }
      htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
    }
  }

  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
     // HAL_TIM_PeriodElapsedCallback(htim);
      if(timer_on_tick!=NULL)
      {
    	  timer_on_tick();
      }
    }
  }
}
void TIM1_UP_IRQHandler(void)
{
	MY_HAL_TIM_IRQHandler(&TIM[0]);
}
void TIM2_IRQHandler(void)
{
	MY_HAL_TIM_IRQHandler(&TIM[1]);
}
void TIM3_IRQHandler(void)
{
	MY_HAL_TIM_IRQHandler(&TIM[2]);
}
void TIM4_IRQHandler(void)
{
	MY_HAL_TIM_IRQHandler(&TIM[3]);
}
/*
#ifdef USE_TIM5
void TIM5_IRQHandler(void)
{
    MY_HAL_TIM_IRQHandler(&TIM[4]);
}
#endif
#ifdef USE_TIM6
void TIM6_IRQHandler(void)
{
    MY_HAL_TIM_IRQHandler(&TIM[5]);
}
#endif
#ifdef USE_TIM7
void TIM7_IRQHandler(void)
{
    MY_HAL_TIM_IRQHandler(&TIM[6]);
}
#endif

#ifdef USE_TIM8
void TIM8_IRQHandler(void)
{
    MY_HAL_TIM_IRQHandler(&TIM[7]);
}
#endif
*/


