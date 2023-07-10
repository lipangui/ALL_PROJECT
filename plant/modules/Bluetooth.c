/*
 * Bluetooth.c
 *
 *  Created on: 2020年5月9日
 *      Author: user
 */
#include "link_protocol.h"
#include "Bluetooth.h"
#include "serial.h"
#include "utils_queue.h"
#include "system.h"
static uint16_t send_number=0;
static uint16_t pack_number=0;
static uint8_t temp=0;
static uint8_t broadcast_message_id=0;
static uint8_t broadcast_head_statues=0;

//这个拆包封帧程序待优化
void ble_pack(frame_config_t *frame_config,uint8_t *buffer,ble_t * ble)
{
	uint16_t buffer_len=0;
	uint16_t len=240;
	if(ble->ble_pack_end==false)
	{
		ble->ble_len=0;
		ble->ble_tx_addr=buffer;
		buffer_len=len_count(buffer);
	}
PACK_PUSH:
		if(ble->ble_pack_addr_flag==true)
		{
			ble->ble_tx_addr=ble->ble_tx_addr+ble->ble_pack_len;
			buffer_len=buffer_len-ble->ble_pack_len;
			pack_number=0;
			ble->ble_pack_addr_flag=false;
		}
		while(frame_config->frame_len)
		{
			if(queue_empty_full(ble->ble_tx_queue)==2)
			{
				return;
			}
			frame_config->frame_len--;
			queue_push(ble->ble_tx_queue,frame_config->frame_buf[pack_number++]);
			ble->ble_len++;
		}

PACK_LOOP1:
	while(buffer_len>=240)
	{
		frame_config->payload=ble->ble_tx_addr;
		frame_config->payload_len=len;
		pack(frame_config);
		if(frame_config->frame_len>240)
		{
			len=len-10;
			goto PACK_LOOP1;
		}
			ble->ble_pack_addr_flag=true;
			ble->ble_pack_len=len;
			ble->ble_pack_end=true;
			goto PACK_PUSH;
	}
	len=buffer_len;
PACK_LOOP2:
	while(buffer_len<240&&buffer_len!=0)
	{
		frame_config->payload=ble->ble_tx_addr;
		frame_config->payload_len=len;
		pack(frame_config);
		if(frame_config->frame_len>240)
		{
			len=len-10;
			goto PACK_LOOP2;
		}
		ble->ble_pack_addr_flag=true;
		ble->ble_pack_len=len;
		ble->ble_pack_end=true;
		goto PACK_PUSH;
	}
	if(buffer_len==0) 	ble->ble_pack_end=false;
}
void ble_send(ble_t * ble,serial_t *serial,uint16_t len)
{
	uint16_t ble_send_number=0;
	uint8_t buffer[ble->ble_tx_queue->size];
	while(len)
	{
		if(queue_empty_full(ble->ble_tx_queue)!=0)
		{
			buffer[ble_send_number++]=queue_pop(ble->ble_tx_queue);
		}
		else	break;
		len--;
	}
	serial_send(serial,buffer,ble_send_number);
	if(len>0&&queue_empty_full(ble->ble_tx_queue)!=0)		ble_send(ble,serial,len);
	if(len==0) 	send_number=0;
}
void ble_parse(frame_t *fram,ble_t *ble)
{
	if(queue_empty_full(ble->ble_rx_queue)!=0)
	{
		if(queue_pop(ble->ble_rx_queue)==0x68)
		{
			ble->ble_rx_status=true;
		}
	}
	switch(ble->ble_rx_status)
	{
	case true:
		while(queue_empty_full(ble->ble_rx_queue)!=0&&(temp<6))
		{
			ble->d_addr[temp++]=queue_pop(ble->ble_rx_queue);
		}
		if(temp==6)
		{
			ble->ble_rx_status=0;
			temp=0;
		}
		else return;
	case false:
		while(fram->parse_state != parse_finish)
		{
			if(queue_empty_full(ble->ble_rx_queue)!=0)
			{
				parse(queue_pop(ble->ble_rx_queue), fram);//解析后的数据存储(包括帧头那些)在fram->data_frame_u.data里
			}
			else break;
		}
		if(fram->parse_state == parse_finish)
		{
			fram->parse_state=parse_state_none;
			memcpy(&ble->ble_buffer[fram->message_id*ble->ble_parse_len_flag],&fram->payload,fram->payload_len);
			ble->ble_parse_len_flag=fram->payload_len;
		}
		else return;
	}
}
void ble_receive(ble_t *ble,serial_t* serial)
{
	if(serial->dma_flag==true)	serial_rail_update(serial);
	while(queue_empty_full(serial->rx_buffer)!=0)
	{
		queue_push(ble->ble_rx_queue,serial_receive(serial));
	}
}
void ble_set_cmd(ble_cmd_t ble_cmd,serial_t *serial,uint8_t *buffer)
{
	uint8_t cmd_buffer[31];
	uint8_t reply_buffer[30];
	uint8_t broadcast=0;
	uint8_t time_temp=system_time/100;//100ms
CMD_LOOP:
	switch(ble_cmd)
	{
		case HOST_SCAN:
					memcpy(&cmd_buffer[0],"<ST_SCAN_DEVICE_ON=1>",21);
					serial_send(serial,cmd_buffer,21);
					break;
		case CLOSE_SCAN:
					memcpy(&cmd_buffer[0],"<ST_SCAN_DEVICE_ON=0>",21);
					serial_send(serial,cmd_buffer,21);
					break;
		case CONNECT_SLAVE:
					memcpy(&cmd_buffer[0],"<ST_CON_MAC=",12);
					memcpy(&cmd_buffer[12],buffer,12);
					memcpy(&cmd_buffer[24],">",1);
					serial_send(serial,cmd_buffer,25);
					break;
		case BREAK_SLAVE:
					memcpy(&cmd_buffer[0],"<ST_CENTER_LINK=",16);
					memcpy(&cmd_buffer[16],buffer,12);
					memcpy(&cmd_buffer[28],">",1);
					serial_send(serial,cmd_buffer,29);
					break;
		case SET_BROADCAST:
					broadcast=len_count(buffer);
					memcpy(&cmd_buffer[0],"<ST_ADV_DATA=",13);
					memcpy(&cmd_buffer[13],buffer,broadcast);
					memcpy(&cmd_buffer[13+broadcast],">",1);
					serial_send(serial,cmd_buffer,14+broadcast);
					break;
	}
		if(time_temp&0x01)
		{
			if(serial_read(serial,reply_buffer)==true)
			{
				switch(ble_cmd)
				{
					case HOST_SCAN:
						if(strstr((char *)reply_buffer,"ok")!=NULL)return;
						else goto CMD_LOOP;
					case SET_BROADCAST:
						if(strstr((char *)reply_buffer,"ok")!=NULL)return;
						else goto CMD_LOOP;
					case CONNECT_SLAVE:
						if(strstr((char *)reply_buffer,(char *)buffer)!=NULL)return;
						else goto CMD_LOOP;
					case BREAK_SLAVE:
						if(strstr((char *)reply_buffer,(char *)buffer)!=NULL)return;
						else goto CMD_LOOP;
					case	CLOSE_SCAN:
						if(strstr((char *)reply_buffer,"cancel")!=NULL)return;
						else goto CMD_LOOP;
				}
			}
		}
}
static void broadcast_pack(broadcast_pack_t *ble_frame,uint8_t payload_len,uint8_t *buffer)
{
	uint8_t len=0;
	uint8_t i=0;
	ble_frame->pack_crc.current=CRC16_INIT_VALUE;
	ble_frame->message_id=broadcast_message_id;
	broadcast_message_id++;
	crc16( ble_frame->message_id,&ble_frame->pack_crc);
	ble_frame->frame_buf[len++]=ble_frame->message_id;
    for (i = 0; i < payload_len; i++)
    {
    	crc16(buffer[i],&ble_frame->pack_crc);
    	ble_frame->frame_buf[len++]=buffer[i];
    }
    	ble_frame->frame_buf[len++]=ble_frame->pack_crc.crc16>>8;
    	ble_frame->frame_buf[len++]=ble_frame->pack_crc.crc16>>0;
    	if((len/2)==1)ble_frame->frame_buf[len++]=0;
    	ble_frame->frame_len=len;
}
static void char_to_hex(uint8_t *in,uint8_t *out,uint8_t len)
{

    for(int j=0;j<len-1;j+=2)
    {
            if(in[j] >= 'a')
            	out[j/2] = in[j] - 'a' + 10;
            else
            	out[j/2] = in[j] - '0';

            if(in[j+1] >= 'a')
            	out[j/2] =(out[j/2] << 4) + in[j+1] - 'a' + 10;
            else
            	out[j/2] =(out[j/2] << 4) + in[j+1] - '0';
    }
    return;
}
static void broadcast_parse(broadcast_frame_t *ble_parse,ble_t *ble)
{
	uint8_t b_addr[12];
	uint8_t b_buffer[32];
	uint8_t status=0;
	uint8_t i=0;
	while(queue_empty_full(ble->ble_rx_queue)!=0)
	{
		if(queue_pop(ble->ble_rx_queue)=='<')status=1;
		if(queue_pop(ble->ble_rx_queue)=='=')status=2;
		if(queue_pop(ble->ble_rx_queue)==',')
		{
			i=0;
			status=3;
		}
		if(queue_pop(ble->ble_rx_queue)==','&&status==3)
		{
			status=4;
			char_to_hex(b_addr,ble->d_addr,12);
			char_to_hex(b_buffer,ble->ble_buffer,i);
			i=0;
		}
       	switch(status)
       	{
       		case 1:	queue_pop(ble->ble_rx_queue);
       					break;
       		case 2:	b_addr[i++]=queue_pop(ble->ble_rx_queue);
       					break;
       		case 3:	b_buffer[i++]=queue_pop(ble->ble_rx_queue);
       					break;
      	}

		}
}
