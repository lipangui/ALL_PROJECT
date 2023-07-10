#include "main.h"
#include "common.h"
#include "system.h"
#include "serial.h"
#include "utils_queue.h"
#include "TFMini.h"
#include "i2c.h"
#include "mpu6050.h"
#include "imu.h"
#include "filter.h"
task_t tasks[3];
system_t *sys;
tfmini_data_t tf_data;
tf_t  tf_filter;
int main(void)
{

	uint32_t time_temp=0,end_time=0;
	system_init(tasks, 3);
	sys = system_get();
	system_start();
	queue_t tx_buffer1;
	queue_t rx_buffer1;

	tx_buffer1.size=100;
	rx_buffer1.size=100;
	queue_init(&tx_buffer1);
	queue_init(&rx_buffer1);

	queue_t tx_buffer2;
	queue_t rx_buffer2;

	tx_buffer2.size=100;
	rx_buffer2.size=100;
	queue_init(&tx_buffer2);
	queue_init(&rx_buffer2);

	serial_t serial_1={serial1,115200,true,&tx_buffer1,&rx_buffer1};
	serial_t serial_2={serial2,115200,true,&tx_buffer2,&rx_buffer2};
	serial_open(&serial_2,NULL);
	serial_open(&serial_1,NULL);
	TFMini_begin(&serial_2,sys);
	IIC_Init();
	Init_MPU6050(sys);
	FilterInit(1000);
	//serial_control_calibration(&serial_1,sys);
  while (1)
  {
	  time_temp=sys->time/10;
	  if(time_temp==end_time) goto END;
	  end_time=time_temp;
	  switch(time_temp%10)
	  {
	  case 0:
		  printf_xyz_tf(&serial_2,&tf_data,&tf_filter);
		  break;
	  default:break;
	  }
	  END:
	  system_delay_us(10);
  }
}
