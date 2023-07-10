/*
 * Bluetooth.h
 *
 *  Created on: 2020年5月9日
 *      Author: user
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_
#include "common.h"
#include "serial.h"
#include "system.h"
#include "common.h"
#define broadcast_len_max 40
typedef enum
{
	//主机功能
	HOST_SCAN=0X01,					//开启扫描（接收广播）
	CLOSE_SCAN=0X02,					//关闭某一从机扫描
	CONNECT_SLAVE=0X04,			//连接从机（点到点通信）
	BREAK_SLAVE=0X08,					//断开从机（先断开从机才能接收到从机的广播）
	//从机功能
	SET_BROADCAST=0X10,			//设置广播内容（内容必须是HEX格式）
}ble_cmd_t;
typedef struct
{
	//接收，解析主要参
	uint8_t ble_buffer[1024];			//解析后最原始的数据存放地方
	uint8_t d_addr[6];						//存放68+mac
	bool ble_rx_status;					//true：表示可以解析了，已经去掉68+mac
	queue_t *ble_rx_queue;				//蓝牙接收队列：异步的关键（开一个3*240字节的缓存区）
	uint16_t ble_parse_len_flag;		//记录上一包的顺序，组包用的

    //发送，封帧主要参
	queue_t *ble_tx_queue;				//蓝牙发送队列：异步的关键（开一个3*240字节的缓存区）  不用一次性把（20k固件包封帧），边封边发送
	bool ble_pack_addr_flag;			//true：封帧时可以地址位移了
	uint8_t *ble_tx_addr;					//记录（20k数据）每一次拆包的地址
	uint16_t ble_pack_len;				//记录每一次拆包的包的长度，下一次地址位移，以及封帧的时候用到
	bool ble_pack_end;					//true：表示（20k数据）全部封帧完了
	uint16_t ble_len;						//每一次发送的长度
	//广播
}ble_t;
typedef struct
{
    uint8_t *frame_buf;
    uint8_t message_id;
    crc16_t pack_crc;
    uint8_t frame_len;
} broadcast_pack_t;
typedef struct
{
	uint8_t data[broadcast_len_max];
    crc16_t crc16;
    uint8_t *payload;
    uint8_t payload_len;
    parse_state_t parse_state;
    uint8_t message_id;
    uint8_t index;
} broadcast_frame_t;
//点到点通信
void ble_pack(frame_config_t *frame_config,uint8_t *buffer,ble_t * ble);//封帧，拆包（240字节一包）
void ble_parse(frame_t *fram,ble_t *ble);//解帧
void ble_receive(ble_t *ble,serial_t* serial);//蓝牙接收
void ble_send(ble_t * ble,serial_t *serial,uint16_t len);//蓝牙发送

//广播通信
void ble_set_cmd(ble_cmd_t ble_cmd,serial_t *serial,uint8_t *buffer);//主机
void ble_receive_broadcast(serial_t *serial,uint8_t *rx_broadcast);
#endif /* BLUETOOTH_H_ */
