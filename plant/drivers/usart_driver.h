/*
 * usart_driver.h
 *
 *  Created on: 2020年4月13日
 *      Author: user
 */

#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_
#include "common.h"

#define SERIAL_STATUS_IDLE  0x00
#define SERIAL_STATUS_IT    0x01
#define SERIAL_STATUS_DMA   0x02

void serial_driver_open(uint32_t baudrate,bool usart_flag,serialx_t port,queue_t *rx_buffer,queue_t *tx_buffer,void (*serial_handler)(void));
void serial_driver_close(serialx_t port);

bool driver_is_busy(serialx_t port);

void serial_driver_set_baudrate(serialx_t port,uint32_t baudratex);

void serial_driver_send(serialx_t port,uint8_t *tx_content,uint16_t len);

bool serial_driver_read(serialx_t port,uint8_t *rx_content);
uint8_t serial_driver_receive(serialx_t port);

void serial_driver_update(serialx_t port);//刷新队尾
#endif /* USART_DRIVER_H_ */
