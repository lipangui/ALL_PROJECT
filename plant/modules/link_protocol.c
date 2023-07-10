#include "link_protocol.h"
#include "crc.h"
#include <stdlib.h>

// 起始字符 0xAA
// 结束字符 0xCC

// 转义字符
// 起始/结束/转义 使用本身即可 当数据中出现0xAA/0xCC/0x0A时候 进行转义
// 起始字符 0xAA = 0x0A 0x0A
// 结束字符 0xCC = 0x0A 0x0C
// 转义字符 0x0A = 0x0A 0x00

//数据帧格式：
//		帧头		版本号		控制位		目标地址		源地址		消息ID		回复ID		数据长度		数据		校验位		帧尾
//	   0XAA		0X10				1				0XFF			 0X00		0X00			 0X00             0				  N 			2			0XCC
#define START           (0xAA)
#define STOP            (0xCC)
#define ESC             (0x0B)
#define ESC_START  0XAA
#define ESC_STOP  	0XCC
#define ESC_ESC  	0X0B

#define PROTOCOL_V10_END        (8)
#define PROTOCOL_V20_END        (12)
#define PROTOCOL_V30_END        (12)
#define PROTOCOL_V40_END        (20)
#define PROTOCOL_V50_END        (32)

#define VERSION_V10             (0x10)
#define VERSION_V20             (0x20)
#define VERSION_V30             (0x30)
#define VERSION_V40             (0x40)
#define VERSION_V50             (0x50)

static char last_char=0xff;
static uint32_t message_id=0;
static void ESC_select(uint64_t ESC_num , uint8_t ESC_len, uint8_t *data, uint32_t *len,crc16_t *crc)
{
    uint8_t temp = 0;

    for (uint8_t i = ESC_len; i > 0; i--)
    {

        temp = ESC_num >> (8 * (i - 1));
        crc16( temp,crc);
        switch (temp)
        {
        case START:
            data[(*len)++] = 0x0B;
            data[(*len)++] = 0xAA;
            break;

        case ESC:
            data[(*len)++] = 0x0B;
            data[(*len)++] = 0x0B;
            break;

        case STOP:
            data[(*len)++] = 0x0B;
            data[(*len)++] = 0xCC;
            break;

        default:
            data[(*len)++] = temp;
            break;
        }
    }
}

void pack(frame_config_t *frame)
{
    uint32_t len = 0;
    uint32_t i = 0;

    frame->pack_crc.current=CRC16_INIT_VALUE;
    // 起始符
    frame->frame_buf[len++] = 0xAA;
    crc16( 0xAA,&frame->pack_crc);
    // 版本号
    ESC_select((uint64_t)frame->version, 1, frame->frame_buf, &len,&frame->pack_crc);

    // 功能符
    ESC_select((uint64_t)frame->controlbits, 1, frame->frame_buf, &len,&frame->pack_crc);

    // 保留符 数据对齐用
    frame->frame_buf[len++] = 0xFF;
    crc16( 0xFF,&frame->pack_crc);
    // 保存 message_id 给外部查看用
    frame->message_id = message_id;

    // 消息id
    switch (frame->version)
    {
    case 0x10:
        ESC_select((uint64_t)frame->dest_addr, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->src_addr, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->message_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->response_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        break;

    case 0x20:
        ESC_select((uint64_t)frame->dest_addr, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->src_addr, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->message_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->response_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->payload_len, 2, frame->frame_buf, &len,&frame->pack_crc);
        frame->frame_buf[len++] = 0xFF;       // 数据对齐 跳字节
        frame->frame_buf[len++] = 0xFF;
        break;

    case 0x30:
        ESC_select((uint64_t)frame->dest_addr, 2, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->src_addr, 2, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->message_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->response_id, 1, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->payload_len, 2, frame->frame_buf, &len,&frame->pack_crc);

        break;

    case 0x40:
        ESC_select((uint64_t)frame->dest_addr, 4, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->src_addr, 4, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->message_id, 2, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->response_id, 2, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->payload_len, 4, frame->frame_buf, &len,&frame->pack_crc);
        break;

    case 0x50:
        ESC_select((uint64_t)frame->dest_addr, 8, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->src_addr, 8, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->message_id, 4, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->response_id, 4, frame->frame_buf, &len,&frame->pack_crc);
        ESC_select((uint64_t)frame->payload_len, 4, frame->frame_buf, &len,&frame->pack_crc);
        break;
    default:
        return;
    }
    message_id++;
    // 数据
    for (i = 0; i < frame->payload_len; i++)
    {
        ESC_select((uint64_t)frame->payload[i], 1, frame->frame_buf, &len,&frame->pack_crc);
    }

    // CRC校验符号
    ESC_select((uint64_t)frame->pack_crc.crc16, 2 , frame->frame_buf, &len,NULL);
    // 结束符
    frame->frame_buf[len++] = 0xCC;
    // 保存 len
    frame->frame_len = len;
}

void parse(char c,frame_t *fram)
{
	uint16_t temp =0;
	if(last_char != ESC)
	{
		if(c == START)	goto RESERT_FRAME;
		if(c == STOP)	goto FINISH_FRAME;
		if(c == ESC)	 {last_char = ESC ;return;}
	}

	fram->data_frame_u.data[fram->index++]=c;
	last_char=0xff;

	if(fram->index>2)
	{
		fram->version = fram->data_frame_u.data[1];
		switch(fram->version)
		{
		case VERSION_V10:
			crc16(fram->data_frame_u.data[fram->index-3],&fram->crc16);
			fram->payload_len=PROTOCOL_V10_END;
			if(fram->index>=PROTOCOL_V10_END)
			{
				fram->payload=&fram->data_frame_u.data[PROTOCOL_V10_END];
				fram->message_id=fram->data_frame_u.data[6];
			}
			break;
		case VERSION_V20:
			if(fram->index==(PROTOCOL_V20_END-2)) fram->index=fram->index+2;
			crc16(fram->data_frame_u.data[fram->index-3],&fram->crc16);
			fram->payload_len=PROTOCOL_V20_END;
			if(fram->index>=PROTOCOL_V20_END)
			{
				fram->payload=&fram->data_frame_u.data[PROTOCOL_V20_END];
				fram->message_id=fram->data_frame_u.data[6];
			}
			break;
		case VERSION_V30:
			crc16(fram->data_frame_u.data[fram->index-3],&fram->crc16);
			fram->payload_len=PROTOCOL_V30_END;
			if(fram->index>=PROTOCOL_V30_END)
			{
				fram->payload=&fram->data_frame_u.data[PROTOCOL_V30_END];
				fram->message_id=fram->data_frame_u.data[8];
			}
			break;
		case VERSION_V40:
			crc16(fram->data_frame_u.data[fram->index-3],&fram->crc16);
			fram->payload_len=PROTOCOL_V40_END;
			if(fram->index>=PROTOCOL_V40_END)
			{
				fram->payload=&fram->data_frame_u.data[PROTOCOL_V40_END];
				fram->message_id=fram->data_frame_u.data[12]<<8|fram->data_frame_u.data[13];
			}
			break;
		case VERSION_V50:
			crc16(fram->data_frame_u.data[fram->index-3],&fram->crc16);
			fram->payload_len=PROTOCOL_V50_END;
			if(fram->index>=PROTOCOL_V50_END)
			{
				fram->payload=&fram->data_frame_u.data[PROTOCOL_V50_END];
				fram->message_id=fram->data_frame_u.data[20]<<24|fram->data_frame_u.data[21]<<16|fram->data_frame_u.data[22]<<8|fram->data_frame_u.data[23];
			}
			break;
		}
	}
	return;

RESERT_FRAME:
				fram->index=0;
				fram->data_frame_u.data[fram->index++]=c;
				fram->crc16.current=CRC16_INIT_VALUE;
				fram->stop=0;
				return;
FINISH_FRAME:
				temp = fram->data_frame_u.data[fram->index - 1] + (fram->data_frame_u.data[fram->index - 2] << 8);
				if(fram->crc16.crc16 != temp)  goto RESERT_FRAME;
				fram->data_frame_u.data[fram->index++]=c;
				fram->stop=c;
				fram->parse_state=parse_finish;
			    fram->version = fram->data_frame_u.data[1];
			    fram->controlbits = fram->data_frame_u.data[2];
			    fram->payload_len=fram->index-3-fram->payload_len;
				return;
}
