/*
 * queue.h
 *
 *  Created on: 2020年4月21日
 *      Author: user
 */
#ifndef __QUEUE_H__
#define __QUEUE_H__
#include "common.h"

void queue_init(queue_t * queue);
uint8_t queue_empty_full(queue_t * queue);
bool queue_push(queue_t * queue, uint8_t byte); //return false when full && not overwritable
uint8_t queue_pop(queue_t * queue); //when empty, always returns 0
void queue_clear(queue_t * queue);  //clear and reset queue
#endif
