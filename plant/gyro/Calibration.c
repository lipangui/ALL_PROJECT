/*
 * Calibration.c
 *
 *  Created on: 2020年6月15日
 *      Author: Curran
 */

#include "Calibration.h"
#include "CalibrationRoutines.h"



/***************加速度计6面矫正，参考APM代码，配合遥控器进行现场矫正**************************/
/***********************************************************
 @函数名：Calibrate_Reset_Matrices
 @入口参数：float dS[6], float JS[6][6]
 @出口参数：无
 @功能描述：矩阵数据复位
 *************************************************************/
void Calibrate_Reset_Matrices(float dS[6], float JS[6][6]) {
	int16_t j, k;
	for (j = 0; j < 6; j++) {
		dS[j] = 0.0f;
		for (k = 0; k < 6; k++) {
			JS[j][k] = 0.0f;
		}
	}
}

/***********************************************************
 @函数名：Calibrate_Find_Delta
 @入口参数：float dS[6], float JS[6][6], float delta[6]
 @出口参数：无
 @功能描述：求解矩阵方程JS*x = dS，第一步把矩阵化上三角阵，
 将JS所在的列下方的全部消为0，然后回代得到线性方程的解
 *************************************************************/
void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6]) {
	//Solve 6-d matrix equation JS*x = dS
	//first put in upper triangular form
	int16_t i, j, k;
	float mu;
	//make upper triangular
	for (i = 0; i < 6; i++) {
		//eliminate all nonzero entries below JS[i][i]
		for (j = i + 1; j < 6; j++) {
			mu = JS[i][j] / JS[i][i];
			if (mu != 0.0f) {
				dS[j] -= mu * dS[i];
				for (k = j; k < 6; k++) {
					JS[k][j] -= mu * JS[k][i];
				}
			}
		}
	}
	//back-substitute
	for (i = 5; i >= 0; i--) {
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0f;

		for (j = 0; j < i; j++) {
			mu = JS[i][j];
			dS[j] -= mu * dS[i];
			JS[i][j] = 0.0f;
		}
	}
	for (i = 0; i < 6; i++) {
		delta[i] = dS[i];
	}
}

void Calibrate_Update_Matrices(float dS[6], float JS[6][6], float beta[6],
		float data[3]) {
	int16_t j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];
	for (j = 0; j < 3; j++) {
		b = beta[3 + j];
		dx = (float) data[j] - beta[j];
		residual -= b * b * dx * dx;//剩余的
		jacobian[j] = 2.0f * b * b * dx;
		jacobian[3 + j] = -2.0f * b * dx * dx;//？
	}

	for (j = 0; j < 6; j++) {
		dS[j] += jacobian[j] * residual;
		for (k = 0; k < 6; k++) {
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

uint8 Calibrate_accel(Acce_Unit accel_sample[6], Acce_Unit *accel_offsets,
		Acce_Unit *accel_scale) {
	int16_t i;
	int16_t num_iterations = 0;
	float eps = 0.000000001;
	float change = 100.0;
	float data[3] = { 0 };
	float beta[6] = { 0 };
	float delta[6] = { 0 };
	float ds[6] = { 0 };
	float JS[6][6] = { 0 };
	bool success = TRUE;
	// reset
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1.0f / GRAVITY_MSS;
	while (num_iterations < 20 && change > eps) {
		num_iterations++;
		Calibrate_Reset_Matrices(ds, JS);
		for (i = 0; i < 6; i++) {
			data[0] = accel_sample[i].x;
			data[1] = accel_sample[i].y;
			data[2] = accel_sample[i].z;
			Calibrate_Update_Matrices(ds, JS, beta, data);
		}
		Calibrate_Find_Delta(ds, JS, delta);
		change = delta[0] * delta[0] + delta[0] * delta[0] + delta[1] * delta[1]
				+ delta[2] * delta[2]
				+ delta[3] * delta[3] / (beta[3] * beta[3])
				+ delta[4] * delta[4] / (beta[4] * beta[4])
				+ delta[5] * delta[5] / (beta[5] * beta[5]);
		for (i = 0; i < 6; i++) {
			beta[i] -= delta[i];
		}
	}
	// copy results out
	accel_scale->x = beta[3] * GRAVITY_MSS;
	accel_scale->y = beta[4] * GRAVITY_MSS;
	accel_scale->z = beta[5] * GRAVITY_MSS;
	accel_offsets->x = beta[0] * accel_scale->x;
	accel_offsets->y = beta[1] * accel_scale->y;
	accel_offsets->z = beta[2] * accel_scale->z;

	printf("%0.2f\r\n",accel_scale->x);
	printf("%0.2f\r\n",accel_scale->y);
	printf("%0.2f\r\n",accel_scale->z);
	printf("%0.2f\r\n",accel_offsets->x);
	printf("%0.2f\r\n",accel_offsets->y);
	printf("%0.2f\r\n",accel_offsets->z);
	// sanity check scale
	if (fabsf(accel_scale->x - 1.0f) > 0.5f
			|| fabsf(accel_scale->y - 1.0f) > 0.5f
			|| fabsf(accel_scale->z - 1.0f) > 0.5f) {
		success = FALSE;
	}
	// sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
	if (fabsf(accel_offsets->x) > 5.0f || fabsf(accel_offsets->y) > 5.0f
			|| fabsf(accel_offsets->z) > 5.0f) {
		success = FALSE;
	}
	// return success or failure
	return success;
}

float Aoco[6] = { 1, 1, 1 };
float Aoc[6][6] = { 1, 1, 1 };
Acce_Unit new_offset = { 0, 0, 0, };
Acce_Unit new_scales = { 1.0, 1.0, 1.0, };

Acce_Unit Accel_Offset_Param = { 0, 0, 0, };
Acce_Unit Accel_Scale_Param = { 0, 0, 0, };
uint8_t Cal_Flag = 0;
void Calibrationer(void) {
	uint16 i = 0;
	Acce_Unit Test_Calibration[6] = { 20, 21, 4152, 4062, -24, 78, -4082, 1, -8,
			-45, -4071, 30, 20, 4035, -8, 30, -60, -3980 };
	for (i = 0; i < 6; i++) {
		Test_Calibration[i].x *= ACCEL_TO_1G;
		Test_Calibration[i].y *= ACCEL_TO_1G;
		Test_Calibration[i].z *= ACCEL_TO_1G;
	}

	Cal_Flag = Calibrate_accel(Test_Calibration, &new_offset, &new_scales);
}

uint8_t imu_direction=6;
uint8_t Accel_Calibration_Flag=0;//加速度计校准模式
uint8_t Accel_Calibration_Finished[6]={0,0,0,0,0,0};//对应面校准完成标志位
uint8_t Accel_Calibration_All_Finished=0;//6面校准全部校准完成标志位
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;

uint8_t cali_firsh_flag = 0;
/***********************************************************
 @函数名：Accel_Calibration_Check
 @入口参数：无
 @出口参数：无
 @功能描述：加速度计标定函数遥控器动作位检测数据
 *************************************************************/
void Accel_Calibration_Check(uint8_t cmd)
{
  uint16_t  i=0;

  if(cali_firsh_flag ==0 )
  {

    Accel_Calibration_Flag=1;//加速度校准模式
    Cal_Flag=0;

    imu_direction=6;
    Accel_Calibration_All_Finished=0;//全部校准完成标志位清零
    Accel_Calibration_Makesure_Cnt=0;
    for(i=0;i<6;i++)
    {
      //Accel_Calibration_Finished[i]=0;//对应面标志位清零
      acce_sample[i].x=0; //清空对应面的加速度计量
      acce_sample[i].y=0; //清空对应面的加速度计量
      acce_sample[i].z=0; //清空对应面的加速度计量
    }
    cali_firsh_flag =1;
  }
  if(Accel_Calibration_Flag==1)
  {
      imu_direction=cmd;
  }

  Accel_Calibartion();//进入校准
}


Acce_Unit acce_sample[6] = { 0 }; //三行6列，保存6面待矫正数据
uint8_t Flash_Buf[12] = { 0 };
/***********************************************************
 @函数名：Accel_Calibartion
 @入口参数：无
 @出口参数：无
 @功能描述：加速度标定、利用遥控器直接进入
 *************************************************************/
uint8_t Accel_Calibartion(void) {
	uint16 i, j = 0;
	float acce_sample_sum[3] = { 0, 0, 0 }; //加速度和数据
	/*第一面，Z轴正向朝着正上方，Z axis is about 1g,X、Y is about 0g*/
	/*第二面，X轴正向朝着正上方，X axis is about 1g,Y、Z is about 0g*/
	/*第三面，X轴正向朝着正下方，X axis is about -1g,Y、Z is about 0g*/
	/*第四面，Y轴正向朝着正下方，Y axis is about -1g,X、Z is about 0g*/
	/*第五面，Y轴正向朝着正上方，Y axis is about 1g,X、Z is about 0g*/
	/*第六面，Z轴正向朝着正下方，Z axis is about -1g,X、Y is about 0g*/
	if (imu_direction <= 5) //检测到对应面数据
	{
		uint16_t num_samples = 0;
		while (num_samples <1000) //采样200次
		{
			Sensor_Update();
			if (Gyro_Length <= 20.0f && IMU_Sensor.imu_updtate_flag == 1) //通过陀螺仪模长来确保机体静止
			{
				for (j = 0; j < 3; j++)
				{
					acce_sample_sum[j] += IMU_Sensor.acce_filter[j]
							* ACCEL_TO_1G; //加速度计转化为1g量程下
				}
				num_samples++;
				IMU_Sensor.imu_updtate_flag = 0;
			}
			Accel_Calibration_Finished[imu_direction] = 1; //对应面校准完成标志位置1
		}
		acce_sample[imu_direction].x = acce_sample_sum[0] / num_samples; //保存对应面的加速度计量
		acce_sample[imu_direction].y = acce_sample_sum[1] / num_samples; //保存对应面的加速度计量
		acce_sample[imu_direction].z = acce_sample_sum[2] / num_samples; //保存对应面的加速度计量
		imu_direction = 6; //单面矫正完毕
	}

	if ((Accel_Calibration_Finished[0] & Accel_Calibration_Finished[1]
			& Accel_Calibration_Finished[2] & Accel_Calibration_Finished[3]
			& Accel_Calibration_Finished[4] & Accel_Calibration_Finished[5])
			&& Accel_Calibration_All_Finished == 0) //6面全部校准完毕
					{
		Accel_Calibration_All_Finished = 1; //加速度计6面校准完成标志
		Accel_Calibration_Flag = 0; //加速度计校准结束，释放遥感操作
		printf("Computing calibration parameters...\r\n");
		Cal_Flag = Calibrate_accel(acce_sample, &new_offset, &new_scales); //将所得6面数据
		for (i = 0; i < 6; i++) {
			Accel_Calibration_Finished[i] = 0; //对应面标志位清零
		}
		if (Cal_Flag == TRUE) //加速度计校准成功
		{

			Accel_Offset_Param.x=new_offset.x;
			Accel_Offset_Param.y=new_offset.y;
			Accel_Offset_Param.z=new_offset.z;
			Accel_Scale_Param.x=new_scales.x;
			Accel_Scale_Param.y=new_scales.y;
			Accel_Scale_Param.z=new_scales.z;

			printf("Six-sided calibration was successful!\n");
			printf("Accel_Offset.x=%f\n",Accel_Offset_Param.x);
			printf("Accel_Offset.y=%f\n",Accel_Offset_Param.y);
			printf("Accel_Offset.z=%f\n",Accel_Offset_Param.z);

			printf("Accel_Scale.x=%f\n",Accel_Scale_Param.x);
			printf("Accel_Scale.y=%f\n",Accel_Scale_Param.y);
			printf("Accel_Scale.z=%f\n",Accel_Scale_Param.z);
			printf("****************\r\n");

			Parameter_Init(); //读取写入参数


		} else //加速度计校准失败
		{
			printf("Six-sided calibration failed!!!\n");
		}
		return TRUE;
	}
	return FALSE;
}

/***********************************************************
 @函数名：Reset_Accel_Calibartion
 @入口参数：uint8_t Type
 @出口参数：无
 @功能描述：加速度标定清空数据与强制复位
 *************************************************************/
void Reset_Accel_Calibartion(uint8_t Type) {
	uint16 i = 0;
	for (i = 0; i < 6; i++) {
		Accel_Calibration_Finished[i] = 0; //对应面标志位清零
		acce_sample[i].x = 0; //清空对应面的加速度计量
		acce_sample[i].y = 0; //清空对应面的加速度计量
		acce_sample[i].z = 0; //清空对应面的加速度计量
	}
	Accel_Calibration_All_Finished = 0; //全部校准完成标志位清零
	if (Type == 1)
		Accel_Calibration_Flag = 0;
}

/***********************************************************
 @函数名：Parameter_Init
 @入口参数：无
 @出口参数：无
 @功能描述：传感器校准参数初始化
 *************************************************************/
bool Parameter_Init(void)
{
	bool success = TRUE;
	/************加速度计零偏与标度值*******/
	// sanity check scale
	if (ABS(Accel_Scale_Param.x - 1.0f) > 0.5f
			|| ABS(Accel_Scale_Param.y - 1.0f) > 0.5f
			|| ABS(Accel_Scale_Param.z - 1.0f) > 0.5f) {
		success = FALSE;
	}
	// sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
	if (ABS(Accel_Offset_Param.x) > 5.0f || ABS(Accel_Offset_Param.y) > 5.0f
			|| ABS(Accel_Offset_Param.z) > 5.0f) {
		success = FALSE;
	}

	if (isnan(Accel_Offset_Param.x) == 0 && isnan(Accel_Offset_Param.y) == 0
			&& isnan(Accel_Offset_Param.z) == 0 && isnan(Accel_Scale_Param.x) == 0
			&& isnan(Accel_Scale_Param.y) == 0 && isnan(Accel_Scale_Param.z) == 0)
					{
		B[0] = Accel_Offset_Param.x;
		B[1] = Accel_Offset_Param.y;
		B[2] = Accel_Offset_Param.z;
		K[0] = Accel_Scale_Param.x;
		K[1] = Accel_Scale_Param.y;
		K[2] = Accel_Scale_Param.z;
	} else if (success == TRUE) {
		B[0] = 0;
		B[1] = 0;
		B[2] = 0;
		K[0] = 1;
		K[1] = 1;
		K[2] = 1;
	}
	return success;
}
///************加速度计6面矫正结束***********************/



uint8_t Gyro_Safety_Calibration_Flag=0;
uint8_t Gyro_Calibration_Check(vector3f *gyro)
{
	static vector3f offset;
	static uint16_t offset_cnt=0;
	static vector3f last_gyro;
	if(gyro->x-last_gyro.x<=50
		   &&gyro->y-last_gyro.y<=50
	      &&gyro->z-last_gyro.z<=50)
	{
	    offset.x+=gyro->x;
		offset.y+=gyro->y;
		offset.z+=gyro->z;
		offset_cnt++;
	}
	else
	{
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
	}
	last_gyro.x=gyro->x;
	last_gyro.y=gyro->y;
	last_gyro.z=gyro->z;

	if(offset_cnt>=200)//持续满足1s
	{
		gyro_offset.x =(offset.x/offset_cnt);//得到标定偏移
		gyro_offset.y =(offset.y/offset_cnt);
		gyro_offset.z =(offset.z/offset_cnt);

	  return 1;
	}
	return 0;
}


void serial_control_calibration(serial_t *serial,system_t *sys)
{
	uint8_t serial_calibration_flag=0;
	uint8_t imu_dir=0;
	uint8_t temp=0;
	uint32_t wait_cali_time=0;
	int time_tips=3;
	while(1)
	{
		serial_rail_update(serial);
		if(serial_calibration_flag==0&&serial_receive(serial)=='0')
		{
			printf("ready!!!go\r\n");
			serial_calibration_flag=1;
		}
		switch(serial_calibration_flag)
		{
		case 1:if(temp==0)
					{
						printf("Orient IMU with Z+ axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		case 2:if(temp==0)
					{
						printf("Orient IMU with X+ axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		case 3:if(temp==0)
					{
						printf("Orient IMU with X- axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		case 4:if(temp==0)
					{
						printf("Orient IMU with Y- axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		case 5:if(temp==0)
					{
						printf("Orient IMU with Y+ axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		case 6:if(temp==0)
					{
						printf("Orient IMU with Z- axis facing up. Send '0'.\r\n");
						temp=1;
					}
					if(serial_receive(serial)=='0')
					{
						printf("Calibrating! This may take a while....\r\n");
						Accel_Calibration_Check(imu_dir);
						printf(" Done.\r\n");
						serial_calibration_flag++;
						imu_dir++;
						temp=0;
					}
					break;
		}
		if(serial_calibration_flag == 0 && (sys->time - wait_cali_time) > 1000) //3秒没接收指令退出
		{
			printf("%d\r\n",time_tips);
			time_tips--;
			if(time_tips <= 0)break;
			wait_cali_time = sys->time;
		}
	}
}
