#include "i2c.h"
#include "mpu6050.h"
#include "gpio.h"
#include "system.h"

Butter_Parameter Accel_Parameter;
Butter_Parameter Gyro_Parameter;
Butter_Parameter Calibrate_Parameter;
Butter_Parameter TF_distance;
Butter_Parameter TF_strenge;




gpiox_t gpio_mpu;
unsigned char mpu_raw_buffer[10];

short int Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb ;
  msb = I2C_ReadOneByte(SlaveAddress,REG_Address);
  lsb = I2C_ReadOneByte(SlaveAddress,REG_Address+1);
  return ( ((short int)msb) << 8 | lsb) ;
}

/***********************************************************
@函数名：Init_MPU6050
@入口参数：无
@出口参数：无
功能描述：MPU6050初始化
*************************************************************/
void Init_MPU6050(system_t *sys)//MPU6050初始化
{
	gpio_pin_t pin={port_b,pin12};
	gpio_mpu.pins=pin;
	gpio_mpu.mode=floating;
	gpio_config(&gpio_mpu,1);

	IICwriteByte(MPU_ADRESS,PWR_MGMT_1  , 0x00);//关闭所有中断,解除休眠
	IICwriteByte(MPU_ADRESS,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
	IICwriteByte(MPU_ADRESS,MPU_CONFIG  , 0x02); //内部低通滤波频率，加速度计94hz，陀螺仪98hz
	IICwriteByte(MPU_ADRESS,MPU_CONFIG  , 0x03);//内部低通滤波频率，加速度计44hz，陀螺仪42hz
	IICwriteByte(MPU_ADRESS,GYRO_CONFIG , 0x10);//1000deg/s
	IICwriteByte(MPU_ADRESS,ACCEL_CONFIG, 0x18);// Accel scale 16g (2048 LSB/g)
	sys->delay(500);
	IMU_Calibration(sys);
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&Gyro_Parameter);//姿态角速度反馈滤波参数  50
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&Accel_Parameter);//姿态解算加计修正滤波值 30
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&Calibrate_Parameter);//传感器校准加计滤波值
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&TF_distance);//传感器校准加计滤波值
	Set_Cutoff_Frequency(Imu_Sampling_Freq, 15,&TF_strenge);//传感器校准加计滤波值

}

/***********************************************************
@函数名：MPU6050_Read_Data
@入口参数：vector3f *gyro,vector3f *accel
@出口参数：无
功能描述：MPU6050数据采集
*************************************************************/
void MPU6050_Read_Data(vector3f *gyro,vector3f *accel)//读取MPU6050数据
{
  accel->x=Double_ReadI2C(MPU_ADRESS,ACCEL_XOUT_H);
  accel->y=-Double_ReadI2C(MPU_ADRESS,ACCEL_YOUT_H);
  accel->z=-Double_ReadI2C(MPU_ADRESS,ACCEL_ZOUT_H);

  gyro->x=Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);
  gyro->y=-Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);
  gyro->z=-Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);
	
}


Vector3f gyro_offset;
s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
/***********************************************************
@函数名：IMU_Calibration
@入口参数：无
@出口参数：无
功能描述：陀螺仪开机零偏标定
*************************************************************/
void IMU_Calibration(system_t *sys)
{
  uint8_t i;
  sys->delay(500);
  for (i = 0; i < 100; i++)			//连续采样30次，一共耗时30*3=90ms
  {
    g_Gyro_xoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_XOUT_H);
    g_Gyro_yoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_YOUT_H);
    g_Gyro_zoffset +=Double_ReadI2C(MPU_ADRESS,GYRO_ZOUT_H);
    sys->delay(10);
  }
  gyro_offset.x =(g_Gyro_xoffset/100);//得到标定偏移
  gyro_offset.y =(g_Gyro_yoffset/100);
  gyro_offset.z =(g_Gyro_zoffset/100);
}

