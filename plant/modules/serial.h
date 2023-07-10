/*
 * serial.h
 *  Created on: 2020年4月10日
 *      Author: user
 */

#ifndef SERIAL_H_
#define SERIAL_H_
#include "common.h"
typedef struct
{
	serialx_t port;				//串口
	uint32_t baudrate;		//波特率
	bool dma_flag;				//是否用dma标志位
	queue_t* tx_buffer;		//发送队列
	queue_t* rx_buffer;		//接收队列
}serial_t;
/*
函数功能：开串口
函数形参：串口结构体，普通中断回调函数（点灯）
函数返回值：无
 */
void serial_open(serial_t* serial,void (*serial_handler)(void));//void (*serial_handler)(void)普通接收中断调用的回调
/*
函数功能：关闭串口
函数形参：串口结构体
函数返回值：无
*/
void serial_close(serial_t* serial);
/*
函数功能：设置波特率
函数形参：串口，波特率
函数返回值：无
*/
void serial_set_baudrate(serialx_t serial, uint32_t baudratex);//设置波特率
/*
函数功能：串口发送
函数形参：串口结构体，发送的buffer，发送的长度
函数返回值：无
*/
void serial_send(serial_t* serial, uint8_t* tx_content, uint16_t len);//根据你前面串口设置才采用dma发送还是普通串口发送
/*
函数功能： 串口接收
函数形参：串口结构体，接收存储的数组
函数返回值：bool型的是否有数据
*/
bool serial_read(serial_t* serial, uint8_t* rx_content);
/*
函数功能：串口接收一个字节
函数形参：串口结构体
函数返回值：一个字节数据
*/
uint8_t serial_receive(serial_t* serial);//读并返回一个字节数据
void serial_handler_1(void);//串口回调函数，用户自己定义
void serial_handler_2(void);
uint16_t len_count(uint8_t *tx_content);//调试用的

//void serial_dma_xx_received(queue_t * fifo); //weak  暂时先放着，待测试
/*
事例演示：
	task_t tasks[3];
	system_t *sys;
	system_init(tasks, 3);
	system_start();
	sys = system_get();//系统时钟初始化

	queue_t tx_buffer;
	queue_t rx_buffer;
	uint8_t tx_content[20]={'1','2','3','4','5'};
	uint8_t *rx_content=(uint8_t *)malloc(sizeof(uint8_t ));

	tx_buffer.size=60;
	rx_buffer.size=60;
	queue_init(&tx_buffer);
	queue_init(&rx_buffer);

	serial_t serial_1={serial1,115200,true,&tx_buffer,&rx_buffer};
	serial_open(&serial_1,serial_handler);//开串口，使能dma

	void on_tick(void)
	{
		serial_read(&serial_1,rx_content);
	}
	timer_timing_t timer={timer1,5000,true,on_tick};
	timer_timing(&timer);//开定时器，5ms调用一次串口接收

	while(1)
	{
		sys->delay(10000);
		serial_send(&serial_1,tx_content,5);
	}
*/
void serial_rail_update(serial_t* serial);//刷新队尾，链路层协议里串口用dma时使用
#endif /* SERIAL_H_ */

