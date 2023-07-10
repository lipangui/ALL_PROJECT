#include "gpio_driver.h"
#include "common.h"

#include "../third_part/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
ADC_HandleTypeDef adc;
DMA_HandleTypeDef dma_adc;
typedef struct
{
	GPIO_TypeDef *gpio;
    uint16_t pin;
    IRQn_Type exit;
    ADC_TypeDef *adc;
    uint32_t dma_buffer_size;
    uint16_t *dma_buffer;
}GPIO_gpio;
GPIO_gpio gpio_t;

typedef struct
{
	uint8_t value_count;
	uint16_t average;
	uint16_t *values;
}adc_cache_t;

adc_cache_t adc_cache[17] = {0};

void (*event_callback[GPIO_PIN_NUM])(void);
void (*adc_dma_callback)(void);

static uint32_t adc_channel_number=1;
static uint32_t dma_event_number=0;

static uint8_t adc_mode_falg=0;
void select_port_pin(gpio_pin_t *pins)
{
	switch(pins->port)
	{
	case port_a: gpio_t.gpio=GPIOA;__HAL_RCC_GPIOA_CLK_ENABLE();break;
	case port_b: gpio_t.gpio=GPIOB;__HAL_RCC_GPIOB_CLK_ENABLE();break;
	case port_c: gpio_t.gpio=GPIOC;__HAL_RCC_GPIOC_CLK_ENABLE();break;
	case port_d: gpio_t.gpio=GPIOD;__HAL_RCC_GPIOD_CLK_ENABLE();break;
	default:break;
	}
	switch(pins->pin)
	{
	case (0x1<<0) : gpio_t.pin=GPIO_PIN_0;
	                gpio_t.exit=EXTI0_IRQn;
	                break;
	case (0x1<<1) : gpio_t.pin=GPIO_PIN_1;
					gpio_t.exit=EXTI1_IRQn;
					break;
    case (0x1<<2) : gpio_t.pin=GPIO_PIN_2;
                    gpio_t.exit=EXTI2_IRQn;
                    break;
	case (0x1<<3) : gpio_t.pin=GPIO_PIN_3;
					gpio_t.exit=EXTI3_IRQn;
					break;
	case (0x1<<4) : gpio_t.pin=GPIO_PIN_4;
					gpio_t.exit=EXTI4_IRQn;
					break;
	case (0x1<<5) : gpio_t.pin=GPIO_PIN_5;
					gpio_t.exit=EXTI9_5_IRQn;
	   	   	   	    break;
	case (0x1<<6) : gpio_t.pin=GPIO_PIN_6;
					gpio_t.exit=EXTI9_5_IRQn;
					break;
	case (0x1<<7) : gpio_t.pin=GPIO_PIN_7;
					gpio_t.exit=EXTI9_5_IRQn;
					break;
	case (0x1<<8) : gpio_t.pin=GPIO_PIN_8;
					gpio_t.exit=EXTI9_5_IRQn;
					break;
	case (0x1<<9) : gpio_t.pin=GPIO_PIN_9;
					gpio_t.exit=EXTI9_5_IRQn;
					break;
	case (0x1<<10): gpio_t.pin=GPIO_PIN_10;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	case (0x1<<11): gpio_t.pin=GPIO_PIN_11;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	case (0x1<<12): gpio_t.pin=GPIO_PIN_12;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	case (0x1<<13): gpio_t.pin=GPIO_PIN_13;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	case (0x1<<14): gpio_t.pin=GPIO_PIN_14;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	case (0x1<<15): gpio_t.pin=GPIO_PIN_15;
					gpio_t.exit=EXTI15_10_IRQn;
					break;
	default:break;
	}
}
void gpio_hw_config(gpio_mode_t mode,gpio_pin_t pins)
{
	GPIO_InitTypeDef GPIO_gpio;
	select_port_pin(&pins);
	switch(mode)
	{
	case port_mode_none:break;
	case pull_down :GPIO_gpio.Mode=GPIO_MODE_INPUT;GPIO_gpio.Pull=GPIO_PULLDOWN;break;
	case pull_up :GPIO_gpio.Mode=GPIO_MODE_INPUT;GPIO_gpio.Pull=GPIO_PULLUP;break;
	case floating :GPIO_gpio.Mode=GPIO_MODE_INPUT;break;
	case open_drain :GPIO_gpio.Mode=GPIO_MODE_OUTPUT_OD;break;
	case push_pull:GPIO_gpio.Mode=GPIO_MODE_OUTPUT_PP;break;
	case af_open_drain :GPIO_gpio.Mode=GPIO_MODE_AF_OD;break;
	case af_push_pull :GPIO_gpio.Mode=GPIO_MODE_AF_PP;break;
	case analog :GPIO_gpio.Mode=GPIO_MODE_ANALOG;break;
	default:break;
	}
	GPIO_gpio.Pin = gpio_t.pin;
	GPIO_gpio.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(gpio_t.gpio, &GPIO_gpio);
}

void gpio_write_bit(gpio_pin_t port_pin, uint8_t bit)
{
	select_port_pin(&port_pin);
	if(bit==0)HAL_GPIO_WritePin(gpio_t.gpio, gpio_t.pin, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(gpio_t.gpio, gpio_t.pin, GPIO_PIN_SET);
}
uint8_t gpio_read_bit(gpio_pin_t port_pin)
{
	select_port_pin(&port_pin);
	return HAL_GPIO_ReadPin(gpio_t.gpio, gpio_t.pin);
}
void gpio_event_enable(gpio_pin_t port_pin, gpio_event_type_t event_type,void (*gpio_event_handler)(void))
{
	GPIO_InitTypeDef GPIO_gpio;
	event_callback[port_pin.pin-1]=gpio_event_handler;
	select_port_pin(&port_pin);
	GPIO_gpio.Pin = gpio_t.pin;
	if(event_type==gpio_rising)GPIO_gpio.Mode=GPIO_MODE_IT_RISING;
	if(event_type==gpio_falling)GPIO_gpio.Mode=GPIO_MODE_IT_FALLING;
	if(event_type==gpio_bothedge)GPIO_gpio.Mode=GPIO_MODE_IT_RISING_FALLING;
	GPIO_gpio.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(gpio_t.gpio, &GPIO_gpio);
	HAL_NVIC_SetPriority(gpio_t.exit, port_pin.pin/4, port_pin.pin%4);
	HAL_NVIC_EnableIRQ(gpio_t.exit);
}
void gpio_event_disable(gpio_pin_t port_pin)
{
	 select_port_pin(&port_pin);
	 HAL_NVIC_DisableIRQ(gpio_t.exit);
}
uint32_t adc_channel_select(gpio_pin_t port_pin)
{
 	if(port_pin.port==port_a)
 	{
 		switch(port_pin.pin)
 		{
 		case pin0: return ADC_CHANNEL_0;break;
 		case pin1: return ADC_CHANNEL_1;break;
 		case pin2: return ADC_CHANNEL_2;break;
 		case pin3: return ADC_CHANNEL_3;break;
 		case pin4: return ADC_CHANNEL_4;break;
 		case pin5: return ADC_CHANNEL_5;break;
 		case pin6: return ADC_CHANNEL_6;break;
 		case pin7: return ADC_CHANNEL_7;break;
 		default:break;
 		}
 	}
 	if(port_pin.port==port_b)
 	{
 	 switch(port_pin.pin)
 	 {
 	   case pin0: return ADC_CHANNEL_8;break;
 	   case pin1: return ADC_CHANNEL_9;break;
 	   default:break;
     }
 	}
 	if(port_pin.port==port_c)
 	 {
 	 	 switch(port_pin.pin)
 	 	 {
 	 	   case pin0: return ADC_CHANNEL_10;break;
 	 	   case pin1: return ADC_CHANNEL_11;break;
 	 	   case pin2: return ADC_CHANNEL_12;break;
 	 	   case pin3: return ADC_CHANNEL_13;break;
 	 	   case pin4: return ADC_CHANNEL_14;break;
 	 	   case pin5: return ADC_CHANNEL_15;break;
 	 	   default:break;
 	     }
 	 }
 	return -1;
}
void adc_select(adcx_t adc)
{
	switch(adc)
	{
	case adc1:gpio_t.adc=ADC1;break;
	case adc2:gpio_t.adc=ADC2;break;
	//case adc3:gpio_t.adc=ADC3;break;
	default:break;
	}
}
void adc_driver_init(void)
{
		RCC_PeriphCLKInitTypeDef ADC_CLKInit;
		ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;            //ADC外设时钟
		ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;            //分频因子6时钟为72M/6=12MHz
		HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);
		__HAL_RCC_ADC1_CLK_ENABLE();

		adc.Init.ScanConvMode = ADC_SCAN_ENABLE;
		adc.Init.ContinuousConvMode = ENABLE;
		adc.Init.DiscontinuousConvMode = DISABLE;
		adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
		adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
}
void gpio_adc_config(adc_t *adc_t, void (*adc_handler)(void))
{
		ADC_ChannelConfTypeDef adc_channel;
		GPIO_InitTypeDef GPIO_gpio;
		select_port_pin(&adc_t->pin);

		adc_driver_init();
		adc.Instance = gpio_t.adc;

		GPIO_gpio.Pin = gpio_t.pin;
		GPIO_gpio.Mode = GPIO_MODE_ANALOG;
		HAL_GPIO_Init(gpio_t.gpio, &GPIO_gpio);

		adc_channel.Channel = adc_channel_select(adc_t->pin);
		adc_channel.Rank = adc_channel_number;
		adc_channel.SamplingTime = adc_t->circle;
		HAL_ADC_ConfigChannel(&adc, &adc_channel);

		adc_cache[adc_channel_number].value_count = adc_t->value_count;
		adc_cache[adc_channel_number].average = adc_t->average;
		adc_cache[adc_channel_number].values = adc_t->values;

		adc_channel_number++;
		if(adc_handler!=NULL)
		{
			adc_dma_callback=adc_handler;
		}

}
void gpio_adc_set_dma_buffer(adcx_t adcx, uint16_t *buffer, uint32_t size)
{
	adc_mode_falg=1;
	__HAL_RCC_DMA1_CLK_ENABLE();
	adc_select(adcx);
	dma_adc.Instance=DMA1_Channel1;
	dma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	dma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_adc.Init.MemInc = DMA_MINC_ENABLE;
    dma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma_adc.Init.Mode = DMA_CIRCULAR;
    dma_adc.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&dma_adc) ;
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	gpio_t.dma_buffer=buffer;
	gpio_t.dma_buffer_size=size;
}
void gpio_adc_start(adcx_t adcx)
{
	adc_channel_number=adc_channel_number-1;
	adc_select(adcx);
	adc.Init.NbrOfConversion= adc_channel_number;
    HAL_ADC_Init(&adc);
	HAL_ADCEx_Calibration_Start(&adc);
	__HAL_LINKDMA(&adc,DMA_Handle,dma_adc);
    if(adc_mode_falg==1)
    {
    HAL_ADC_Start_DMA(&adc,(uint32_t*)gpio_t.dma_buffer,gpio_t.dma_buffer_size);//这个uint_t32和uint_t16要注意
    }

}

void gpio_adc_stop(adcx_t adcx)
{
	HAL_ADC_Stop(&adc);
}


uint16_t gpio_adc_once(adcx_t adcx,gpio_pin_t pin, uint8_t circle)
{
	ADC_InjectionConfTypeDef adc_Injected;
	GPIO_InitTypeDef GPIO_gpio;
	select_port_pin(&pin);
	adc_select(adcx);

	GPIO_gpio.Pin = gpio_t.pin;
	GPIO_gpio.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(gpio_t.gpio, &GPIO_gpio);

	adc.Instance =gpio_t.adc;
	adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	adc.Init.ContinuousConvMode = DISABLE;
	adc.Init.DiscontinuousConvMode = DISABLE;
	adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc.Init.NbrOfConversion = 1;
	HAL_ADC_Init(&adc);
	adc_Injected.InjectedChannel = adc_channel_select(pin);
	adc_Injected.InjectedRank = ADC_INJECTED_RANK_1;
	adc_Injected.InjectedNbrOfConversion = 1;
	adc_Injected.InjectedSamplingTime = circle;
	adc_Injected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	adc_Injected.AutoInjectedConv = DISABLE;
	adc_Injected.InjectedDiscontinuousConvMode = DISABLE;
	adc_Injected.InjectedOffset = 0;
	HAL_ADCEx_InjectedConfigChannel(&adc, &adc_Injected);
	HAL_ADCEx_Calibration_Start(&adc);
	HAL_ADC_Start(&adc);
	HAL_ADC_PollForConversion(&adc,10);
	return (uint16_t)HAL_ADC_GetValue(&adc);
}
void adc_dma_handler(void)
{
	uint8_t adc_channel=0;
	uint32_t sum;
	uint8_t adc_smoothing=0;
		for(adc_channel = 1; adc_channel <=adc_channel_number; adc_channel++)
				{
			       dma_event_number = adc_channel-1;
					 sum=0;
					for(adc_smoothing = 0; adc_smoothing < adc_cache[adc_channel].value_count; adc_smoothing++)
					{
						adc_cache[adc_channel].values[adc_smoothing] = gpio_t.dma_buffer[dma_event_number];
						sum+=gpio_t.dma_buffer[dma_event_number];
						dma_event_number += adc_channel_number;
						adc_cache[adc_channel].average=(sum/adc_cache[adc_channel].value_count);
					}
				}
}



void gpio_driver_write(gpio_value_def_t gpio_value_def, uint32_t value)
{
	GPIO_InitTypeDef GPIO_gpio;
	gpio_pin_t *set_gpio=gpio_value_def.gpio_mapping;
  	uint8_t value_number;
  	if(gpio_value_def.value_type==gpio_8bit_value)value_number=8;
  	if(gpio_value_def.value_type==gpio_16bit_value)value_number=16;
  	if(gpio_value_def.value_type==gpio_32bit_value)value_number=32;
  	while(value_number)
  	{
  		value_number--;
  		select_port_pin(set_gpio);
  		GPIO_gpio.Pin = gpio_t.pin;
  		if(((set_gpio->port==port_a)&&(set_gpio->pin==(pin13|pin14|pin15)))||((set_gpio->port==port_b)&&(set_gpio->pin==(pin3|pin4))) )
  		{
  			__HAL_AFIO_REMAP_SWJ_NONJTRST();
  			GPIO_gpio.Mode=GPIO_MODE_AF_PP;
  		}
  		else GPIO_gpio.Mode=GPIO_MODE_OUTPUT_PP;
  		GPIO_gpio.Speed=GPIO_SPEED_FREQ_LOW;
  		HAL_GPIO_Init(gpio_t.gpio, &GPIO_gpio);
  		HAL_GPIO_WritePin(gpio_t.gpio, gpio_t.pin,(value&0x01));
  		value=(value>>1);
  		set_gpio++;
  	}

}
uint32_t gpio_driver_read(gpio_value_def_t gpio_value_def)
{
	gpio_pin_t *read_gpio=gpio_value_def.gpio_mapping;
	uint32_t value=0;
	uint8_t value_number;
	uint8_t i=0;
	if(gpio_value_def.value_type==gpio_8bit_value)value_number=8;
	if(gpio_value_def.value_type==gpio_16bit_value)value_number=16;
	if(gpio_value_def.value_type==gpio_32bit_value)value_number=32;
   while(value_number)
   {
	   value_number--;
	   select_port_pin(read_gpio);
	   value|=(HAL_GPIO_ReadPin(gpio_t.gpio,gpio_t.pin)<<i);
	   i++;
	   read_gpio++;
   }
   return value;
}

void MY_HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
	 if (__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != 0x00u)
	  {
	    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
	    if(event_callback[GPIO_Pin-1]!=NULL)
	    {
	      event_callback[GPIO_Pin-1]();
	    }
	  }
}
void MY_HAL_HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
	if ((((hdma->DmaBaseAddress->ISR) & (DMA_FLAG_TC1 << hdma->ChannelIndex)) != RESET) && (((hdma->Instance->CCR) & DMA_IT_TC) != RESET))
		{
			__HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
			adc_dma_handler();
			if(adc_dma_callback!=NULL)
			{
				adc_dma_callback();
			}
		}
}
void EXTI0_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
void EXTI1_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}
void EXTI3_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}
void EXTI9_5_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
}
void EXTI15_10_IRQHandler(void)
{
	MY_HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
}
void DMA1_Channel1_IRQHandler(void)
{
	MY_HAL_HAL_DMA_IRQHandler(&dma_adc);
}
