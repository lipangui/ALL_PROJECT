#include "system_driver.h"
#include "../third_part/CMSIS/Device/ST/STM32F1xx/Include/stm32f1xx.h"
volatile uint32_t time_temp;
static volatile uint32_t usTicks = 0;
void system_driver_start(void)
{
	HAL_ResumeTick();
	__enable_irq();
}
void system_driver_init(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
	usTicks=HAL_RCC_GetHCLKFreq()/sys_1us;
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/sys_1ms);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}
void system_driver_delay(uint32_t ms)
{

    uint32_t target;
    target = time_temp + ms;
    while (time_temp < target);
}

void SysTick_Handler(void)
{
		time_temp++;
		systick_handler();
}
__attribute__((weak)) void systick_handler(void)
{
    ;
}
void system_driver_delay_us(uint32_t us)
{
	uint32_t tcnt=sys_timer_get_micros();
	while(sys_timer_get_micros() - tcnt < us);
}
uint32_t sys_timer_get_micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = time_temp;
        cycle_cnt = SysTick->VAL;
    	} while (ms != time_temp);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;  //us
}
