#ifndef SYSTEM_DRIVER_H_
#define SYSTEM_DRIVER_H_
#include <stdint.h>

#define sys_1ms   1000
#define sys_1us    1000000

void system_driver_start(void);
void system_driver_init(void);
void system_driver_delay(uint32_t ms);
uint32_t  system_driver_get_timer(void);
void systick_handler(void);
uint32_t sys_timer_get_micros(void);
void system_driver_delay_us(uint32_t us);
#endif /* SYSTEM_DRIVER_H_ */
