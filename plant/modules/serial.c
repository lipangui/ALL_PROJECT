/*
 * uart.c
 *
 *  Created on: 2020年4月10日
 *      Author: user
 */
#include "serial.h"
#include "usart_driver.h"

serial_t serial[serial_max];
uint8_t k=0;
void serial_handler_1(void)
{
	k++;
}
void serial_handler_2(void)
{
	;
}
static bool is_busy(serialx_t serialx)//本地内部调用函数，在发送，接收判断
{
    if(driver_is_busy(serialx)==true)	return true;
    else  return false;
}
void serial_open(serial_t* serial,void (*serial_handler)(void))
{
	serial_driver_open(serial->baudrate,serial->dma_flag,serial->port,serial->rx_buffer,serial->tx_buffer,serial_handler);
}

void serial_set_baudrate(serialx_t serial, uint32_t baudratex)
{
    serial_driver_set_baudrate(serial,baudratex);
}
void serial_send(serial_t* serial, uint8_t* tx_content, uint16_t len)
{
	if(is_busy(serial->port)!=true)
	{
		serial_driver_send(serial->port,tx_content,len);
	}
}
bool serial_read(serial_t* serial, uint8_t* rx_content)
{
  if(serial_driver_read(serial->port,rx_content)==true) return true;
  else return false;
}
uint8_t serial_receive(serial_t* serial)//读并返回一个字节数据
{
   return serial_driver_receive(serial->port);
}
void serial_rail_update(serial_t* serial)
{
	serial_driver_update(serial->port);
}
uint16_t len_count(uint8_t *tx_content)
{
	uint8_t *buffer=tx_content;
	uint16_t count=0;
	while(*buffer)
	{
		count++;
		buffer++;
	}
	return count;
}


