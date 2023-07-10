#include "system.h"
#include "system_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

uint16_t system_time=0;
system_t systems;
void system_init(task_t* tasks, uint32_t task_count)
{
	uint8_t i=0;
	system_driver_init();
	systems.time=0;
#if USE_FreeRTOS
	systems.delay=vTaskDelay;
	for(i=0;i<task_count;i++)
	{
		if(system_TaskCreate(tasks[i]));
	}
#else
	systems.delay=system_driver_delay;
#endif
}
void system_start(void)
{
#if USE_FreeRTOS
	vTaskStartScheduler();
#endif
	system_driver_start();
}
#if USE_FreeRTOS
uint8_t system_TaskCreate(task_t task)//创建任务
{

	taskENTER_CRITICAL();
    if( pdPASS!= xTaskCreate( task.task_func, task.name, task.stack_size, task.parameters, task.priority,task.handle ))
    {
  	taskEXIT_CRITICAL();
	return 1;
    }
    else return 0;
}
void system_TaskSuspend(task_Handle_t *handle)//挂起
{
	vTaskSuspend(*handle);
}
void system_TaskResume(task_Handle_t *handle)//恢复
{
	vTaskResume(*handle);
}
void system_TaskDelete(task_Handle_t *handle)//删除任务
{
	vTaskDelete(*handle);
}
system_QueueHandle_t system_queueCreate(uint16_t queue_len, uint16_t item_size)//创建队列
{
	return (system_QueueHandle_t)xQueueCreate(queue_len, item_size);
}
void system_QueueDelete(system_QueueHandle_t queue)//删除队列
{
	vQueueDelete(queue);
}
system_BaseType_t system_queueSend(system_QueueHandle_t queue, void* send, uint16_t wait_ms,bool use_isr)//队列发送数据
{
	system_BaseType_t xHigherPriorityTaskWoken;
    uint32_t ticks_to_wait = wait_ms / portTICK_PERIOD_MS;
    if(use_isr==false)
    {
    	if(xQueueSend(queue,  send, ticks_to_wait) == pdTRUE)	 return 1;
    }
    if(use_isr== true)
    {
    	if(xQueueSendFromISR(queue,  send, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			return 1;
		}
    }
    return 0;
}
system_BaseType_t system_queueReceive(system_QueueHandle_t queue, void* receive, uint16_t wait_ms,bool use_isr)//队列接收数据
{
	system_BaseType_t xHigherPriorityTaskWoken;
	uint32_t ticks_to_wait = wait_ms * portTICK_PERIOD_MS;
	if(use_isr==false)
	{
		if(xQueueReceive(queue, receive, ticks_to_wait) == pdTRUE) return 1;
	}
	if(use_isr==true)
	{
		if(xQueueReceiveFromISR(queue, receive, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			return 1;
		}
	}
 	return 0;
}
semaphore_Handle_t system_semaphore_Create(semaphore_t semaphore)
{
	switch(semaphore.semaphores)
	{
	case mutex:
		return xSemaphoreCreateMutex();
		break;
	case counting_semaphore:
		return xSemaphoreCreateCounting(semaphore.len,semaphore.size);
		break;
	case recursive_semaphore:
		return xSemaphoreCreateRecursiveMutex();
		break;
	case binary_semaphore:
		return xSemaphoreCreateBinary();
		break;
	}
	return NULL;
}
void system_SemaphoreDelete(semaphore_Handle_t semaphore)
{
	vSemaphoreDelete(semaphore);
}
system_BaseType_t system_SemaphoreGive(semaphore_Handle_t semahpore,bool use_isr)
{
	system_BaseType_t xHigherPriorityTaskWoken;
	if(use_isr==true)
	{
		if(xSemaphoreGiveFromISR(semahpore,&xHigherPriorityTaskWoken)==pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			return 1;
		}
	}
	if(use_isr==false)
	{
		if(xSemaphoreGive(semahpore)==pdTRUE) return 1;
	}
	return 0;
}
system_BaseType_t system_SemaphoreTake(semaphore_Handle_t semahpore,bool use_isr,uint16_t block_time)
{
	system_BaseType_t xHigherPriorityTaskWoken;
	if(use_isr==true)
	{
		if(xSemaphoreTakeFromISR (semahpore,&xHigherPriorityTaskWoken)==pdTRUE)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			return 1;
		}
	}
	if(use_isr==false)
	{
		if(xSemaphoreTake(semahpore,block_time)==pdTRUE) return 1;
	}
	return 0;
}
#endif
void systick_handler(void)
{
	system_time++;
    systems.time++;
#if USE_FreeRTOS
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
#endif
	xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	}
#endif
#endif
}
system_t * system_get(void)
{
	return &systems;
}
void system_delay_us(uint32_t us)
{
	system_driver_delay_us(us);
}
uint32_t sys_timer_get_micros_module(void)
{
	uint32_t temp=sys_timer_get_micros();
	return temp;
}
