/*
 * demo_queue.c
 *
 *  Created on: 2020年4月9日
 *      Author: user
 */
#include <utils_queue.h>
#include "common.h"
void queue_init(queue_t * queue)
{
 	queue->buffer=(uint8_t *)malloc(sizeof(uint8_t)*queue->size);
    if (queue->buffer == NULL)
    {
        perror("malloc error: ");
        exit(1);
    }
    memset(queue->buffer,0,sizeof(uint8_t)*queue->size);
 	queue->rail=queue->front=0;
 	queue->overwritable=true;
 	queue->overflow=false;
 	queue->full=false;
 	queue->empty=true;
}
uint8_t queue_empty_full(queue_t * queue)
{
	if(queue->front==queue->rail && queue->overflow==false)
		{
		queue->empty=true;
		return 0;
		}
	 if ( queue->rail==queue->front && queue->overflow==true )
	{
		queue->full=true;
		return 2;
	}
	 else
		{
			queue->empty=false;
			 queue->full=false;
			return 1;
		}
}
bool queue_push(queue_t * queue, uint8_t byte) //return false when full && not overwritable
{
		if((queue->overwritable==false)&&(queue->full==true))
		{
			return false;
		}
		else
		{
			if(queue->full==true&&queue->overwritable==true) queue->front=(queue->front+1)%queue->size;
			queue->buffer[queue->rail]=byte;
			queue->rail=(queue->rail+1)%queue->size;
			if(queue->rail==queue->front) queue->overflow=true;
			return true;
		}
}
uint8_t queue_pop(queue_t * queue)//when empty, always returns 0
{
	uint8_t value;
	if(queue->empty==true)
	{
		return -1;
	}
	else
	{
    	value=queue->buffer[queue->front];
		queue->front=(queue->front+1)%queue->size;
		if(queue->front==queue->rail) queue->overflow=false;
		return value;
	}
}
void queue_clear(queue_t * queue)
{
	queue->rail=queue->front=0;
	queue->empty=true;
	queue->full=false;
}
