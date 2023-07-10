#include "TFMini.h"
#include "common.h"
#include "serial.h"
#include "utils_queue.h"


#define NEXT_HEAD		8
#define READ_DATA 		9
#define READ_HEAD         0
#define READ_DIST_L       1
#define READ_DIST_H       2
#define READ_STRENGTH_L   3
#define READ_STRENGTH_H   4
#define READ_TEMP_L       5
#define READ_TEMP_H       6
#define READ_CHEKSUM      7

tfmini_data_t tfdata;
tf_t last_tf;
/*float TF_Data[2][N2];*/

int TFMini_begin(serial_t *serial,system_t *sys)
{

	tfdata.state = READY;
	tfdata.distance = -1;
	tfdata.strength = -1;
	tfdata.temperature =-1;

	TFMini_setOutputFormat(serial);
	TFMini_setOutputRate(100,serial);  //100hz
	//TFMini_setBound(115200,serial);
	TFMini_setOutputPreservation(serial);
	sys->delay(2000);
  return tfdata.state;
}


void TFMini_setOutputFormat(serial_t *serial) {
  // Set to "millimeter" output mode
	uint8_t data[8]={0};
	data[0]=0x5A;
	data[1]=0x05;
	data[2]=0x05;
	data[3]=0x06;
	data[4]=0x6A;
	//write(SERIAL2,data,5);
	serial_send(serial,data,5);
}
void TFMini_setOutputPreservation(serial_t *serial)
{
	uint8_t data[4]={0};
	data[0]=0x5A;
	data[1]=0x04;
	data[2]=0x11;
	data[3]=0x6F;
	serial_send(serial,data,4);
}

void TFMini_setOutputRate(uint16_t rate,serial_t *serial) {
	
	
	uint8_t data[6]={0};
	data[0]=0x5A;
	data[1]=0x06;
	data[2]=0x03;
	data[3]=rate;
	data[4]=rate >> 8;
	data[5]=(data[0]+data[1]+data[2]+data[3]+data[4])& 255;
	serial_send(serial,data,6);
}
void TFMini_setBound(uint32_t bound,serial_t *serial)
{
	uint8_t data[8]={0};
	data[0]=0x5A;
	data[1]=0x08;
	data[2]=0x06;
	data[3]=bound ;
	data[4]=bound >> 8;
	data[5]=bound >> 16;
	data[6]=bound >> 24;
	data[7]=(data[0]+data[1]+data[2]+data[3]+data[4]+data[5]+data[6])& 255;
	serial_send(serial,data,8);
}




// Private: Handles the low-level bits of communicating with the TFMini, and detecting some communication errors.
tfmini_data_t TFMini_getData(serial_t *serial,uint8_t *frame) {
	int numCharsRead = 0;
	uint8_t status =0;
	uint8_t checksum = 0x59 + 0x59;
	uint8_t lastChar = 0x00, curChar = 0x00;  
	// Step 1: Read the serial stream until we see the beginning of the TF Mini header, or we timeout reading too many characters.
	while (1) {
		serial_rail_update(serial);
		if (queue_empty_full(serial->rx_buffer)!=0)
		{    
			curChar = serial_receive(serial);
			if(status == 0)
			{
				if ((lastChar == 0x59) && (curChar == 0x59)) {
					status++;
				} 
				else {
				// We have not seen two 0x59's in a row -- store the current character and continue reading.         
				lastChar = curChar;
				curChar =0x00;
				numCharsRead += 1; 
				}           
				
				if (numCharsRead > TFMINI_MAXBYTESBEFOREHEADER) {
					tfdata.state = ERROR_SERIAL_NOHEADER;
					tfdata.distance = -1;
					tfdata.strength = -1;
					tfdata.temperature = -1;
					return tfdata;      
				}
			}else if(status == READ_DIST_L)
			{
				frame[0] = 0x59;
				frame[1] = 0x59;
				frame[2] = curChar;
				checksum += frame[2];
				status++;
			}
			else if(status == READ_DIST_H)
			{
				frame[3] = curChar;
				checksum += frame[3];
				status++;
			}
			else if(status == READ_STRENGTH_L)
			{
				frame[4] = curChar;
				checksum += frame[4];
				status++;
			}
			else if(status == READ_STRENGTH_H)
			{
				frame[5] = curChar;
				checksum += frame[5];
				status++;
			}
			else if(status == READ_TEMP_L)
			{
				frame[6] = curChar;
				checksum += frame[6];
				status++;
			}			
			else if(status == READ_TEMP_H)
			{
				frame[7] = curChar;
				checksum += frame[7];
				status++;
			}
			else if(status == READ_CHEKSUM)
			{
				frame[8] = curChar;
				break;
			}

		}
			
	}
	
	//// Step 2A: Compare checksum
		uint8_t checksumByte = frame[8];
		if (checksum != checksumByte) {
			tfdata.state = ERROR_SERIAL_BADCHECKSUM;
			tfdata.distance = -1;
			tfdata.strength = -1;
			tfdata.temperature = -1;
			return tfdata;
		}
	
	// Step 4: Store values
	tfdata.distance = (frame[3] << 8) + frame[2];
	tfdata.strength = (frame[5] << 8) + frame[4];
	tfdata.temperature = (frame[ 7]<< 8)+frame[ 6];
	tfdata.state = MEASUREMENT_OK;


	return tfdata;  
}
static void TFMini_parse_Data(tfmini_data_t *tf_data,uint8_t input_data)
{
	if(input_data==0x59&&tf_data->state==READ_HEAD) goto TF_READY;
	if(tf_data->index>=8)goto TF_END;
	tf_data->check_sum+=input_data;
	tf_data->data_buffer[tf_data->index++]=input_data;
	return;
TF_READY:
			tf_data->check_sum=0;
			tf_data->index=0;
			tf_data->check_sum+=input_data;
			tf_data->data_buffer[tf_data->index++]=input_data;
			tf_data->state=NEXT_HEAD;
			return;
TF_END:
			if((input_data&0xff)!=(tf_data->check_sum&0xff))goto TF_READY;
			if((tf_data->data_buffer[0]!=0x59)||(tf_data->data_buffer[1]!=0x59))goto TF_READY;
			tf_data->data_buffer[tf_data->index]=input_data;
			tf_data->distance=(tf_data->data_buffer[3]<<8)+tf_data->data_buffer[2];
			tf_data->strength=(tf_data->data_buffer[5]<<8)+tf_data->data_buffer[4];
			tf_data->temperature=(tf_data->data_buffer[7]<<8)+tf_data->data_buffer[6];
			tf_data->state=MEASUREMENT_OK;
			return;
}
bool TFMini_receive_Data(serial_t *serial,tfmini_data_t *tf_data)
{
	while(tf_data->state!=MEASUREMENT_OK)
	{
		serial_rail_update(serial);
		if(queue_empty_full(serial->rx_buffer)!=0)
		{
			TFMini_parse_Data(tf_data,serial_receive(serial));
		}
	}
	if(tf_data->state==MEASUREMENT_OK)
	{
		tf_data->state=READ_HEAD;
		return true;
	}
	return false;
}
/*void TFMini_Fiter_Data(serial_t *serial,tfmini_data_t *tf_data,tf_t *tf_filter)
{
	if(TFMini_receive_Data(serial,tf_data)==true)
	{
		tf_filter->distance=GildeAverageValueFilter_MAG((float)tf_data->distance,TF_Data[0]);
		tf_filter->strength=GildeAverageValueFilter_MAG((float)tf_data->strength,TF_Data[1]);
	}
}*/

