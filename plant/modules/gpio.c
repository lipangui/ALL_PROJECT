#include "gpio_driver.h"
#include "common.h"
#include "gpio.h"

void gpio_event_handler(void)
{
		;
}
void adc_handler(void)
{
		;
}
void gpio_config(gpiox_t *gpioes, uint8_t len)//len指全部引脚数
{
	while(len)
	{
     len--;
   	 gpio_hw_config(gpioes->mode,gpioes->pins);
   	 gpioes++;
	}
}


void gpio_write(gpio_value_def_t gpio_value_def, uint32_t value)
{
	gpio_driver_write(gpio_value_def,value);
}
uint32_t gpio_read(gpio_value_def_t gpio_value_def)
{
	return gpio_driver_read(gpio_value_def);
}


