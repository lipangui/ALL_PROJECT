/* - - - - - - - - -  TFMini  - - - - - - - - -
  Data Frame format:
  Byte0  Byte1  Byte2   Byte3   Byte4   Byte5   Byte6   Byte7   Byte8
  0x59   0x59   Dist_L  Dist_H  Flux_L  Flux_H  Temp_L  Temp_H  CheckSum_
  Data Frame Header character: Hex 0x59, Decimal 89, or "Y"

  Command format:
  Byte0  Byte1   Byte2   Byte3 to Len-2  Byte Len-1
  0x5A   Length  Cmd ID  Payload if any   Checksum
 - - - - - - - - - - - - - - - - - - - - - - - - - */

// This library's 'sendCommand( cmnd, param)' function
// defines a command (cmnd) in the the following format:
// 0x     00       00       00       00
//     one byte  command  command   reply
//     payload   number   length    length

#ifndef _TF_MINI_H_
#define _TF_MINI_H_


#include "common.h"
#include "serial.h"
#include "system.h"

//#include "usart_bsp.h"



// The frame size is nominally 9 characters, but we don't include the first two 0x59's marking the start of the frame
#define TFMINI_FRAME_SIZE                 7

// Timeouts
#define TFMINI_MAXBYTESBEFOREHEADER       30
#define TFMINI_MAX_MEASUREMENT_ATTEMPTS   10

// States
#define READY                             0
#define ERROR_SERIAL_NOHEADER             1
#define ERROR_SERIAL_BADCHECKSUM          2
#define ERROR_SERIAL_TOOMANYTRIES         3
#define MEASUREMENT_OK                    10
/*#define N2 					10*/


extern int state;
/*extern float TF_Data[2][N2];*/
typedef struct 
{
	uint16_t distance;
	uint16_t strength;
	uint16_t temperature;
	int state;
	uint16_t check_sum;
	uint8_t data_buffer[9];
	uint8_t index;
}tfmini_data_t;

typedef struct
{
	float distance;
	float strength;
}tf_t;
extern tf_t last_tf;
int TFMini_begin(serial_t *serial,system_t *sys);
void TFMini_setOutputFormat(serial_t *serial);
void TFMini_setOutputRate(uint16_t rate,serial_t *serial);
void TFMini_setOutputPreservation(serial_t *serial);
void TFMini_setBound(uint32_t bound,serial_t *serial);
tfmini_data_t TFMini_getData(serial_t *serial,uint8_t *frame);
bool TFMini_receive_Data(serial_t *serial,tfmini_data_t *tf_data);
void TFMini_Fiter_Data(serial_t *serial,tfmini_data_t *tf_data,tf_t *tf_filter);
#endif //_TF_MINI_H_
