#ifndef SYSTEM_H_
#define SYSTEM_H_
#include <stdint.h>
#include "common.h"
#define USE_FreeRTOS 0

extern uint16_t system_time;
typedef long system_BaseType_t;
typedef void * system_QueueHandle_t;//队列句柄
typedef void * task_Handle_t;//任务句柄
typedef void * semaphore_Handle_t;//信号量句柄
typedef struct
{
    const char *name;
    void (*task_func)(void*);
    uint16_t stack_size;
    void *parameters;
    uint32_t priority;
    task_Handle_t handle;//任务句柄
} task_t;//任务结构体
typedef struct
{
	uint16_t time;
	void (*delay)(uint32_t);
}system_t;
typedef struct
{
	semaphore_mode_t semaphores;
	uint16_t len;
	uint16_t size;
}semaphore_t;
void system_init(task_t* tasks, uint32_t task_count);
system_t * system_get(void);
void system_start(void);
void systick_handler(void);
uint32_t sys_timer_get_micros_module(void);
void system_delay_us(uint32_t us);
#if USE_FreeRTOS
uint8_t system_TaskCreate(task_t  tasks);
void system_TaskSuspend(task_Handle_t *handle) ;
void system_TaskResume(task_Handle_t *handle);//这两个用的地方不多，可以考虑省略
void system_TaskDelete(task_Handle_t *handle);//删除任务

system_QueueHandle_t system_queueCreate(uint16_t queue_len, uint16_t item_size);
void system_QueueDelete(system_QueueHandle_t queue);
system_BaseType_t system_queueSend(system_QueueHandle_t queue, void* send, uint16_t wait_ms,bool use_isr);
system_BaseType_t system_queueReceive(system_QueueHandle_t queue, void* receive, uint16_t wait_ms,bool use_isr);//线程通信

semaphore_Handle_t system_semaphore_Create(semaphore_t semaphore);
void system_SemaphoreDelete(semaphore_Handle_t semaphore);
system_BaseType_t system_SemaphoreGive(semaphore_Handle_t semahpore,bool use_isr);
system_BaseType_t system_SemaphoreTake(semaphore_Handle_t semahpore,bool use_isr,uint16_t block_time);
#endif

#endif /* SYSTEM_H_ */
