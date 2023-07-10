#include "imu.h"

#define filter_number 20
Sensor IMU_Sensor;
float xyz_data_buffer[3][N2];
float tf_data_buffer[2][N2];
#define Kp 100.0f                        // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.001f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f                // 采样周期的一半
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差
float Yaw_t,Pitch_t,Roll_t;  //偏航角，俯仰角，翻滚角
float SIN_PITCH=0.0;
#define  YAW_FUSION_MODE_QUAD  1	//1：偏航角来源于四元数更新，时间长容易偏移，
//0：偏航角来源于偏航角速度与磁力计一阶互补，GPS定点时选择此方式
//   推荐采用外置磁力计数据
volatile float Pitch, Roll, Yaw;								//俯仰角、横滚角、偏航角
volatile float Yaw_Gyro = 0, Pitch_Gyro = 0, Roll_Gyro = 0;	//俯仰角速度、横滚角速度、偏航角速度
float Pitch_Observation, Roll_Observation, Yaw_Observation;	//俯仰观测角、横滚观测角、偏航观测角
float Inclination_Rate = 0;
Vector3f_Body Circle_Angle;
float Gyro_Length = 0;													//陀螺仪模长
float K[3] = {1.005800,1.003127,0.989342};										//默认标度(量程)误差
float B[3] = {0.257974,0.038208,1.059505};												//默认零位误差
/*
float K[3] = { 1.016399,1.028530,0.994706};										//默认标度(量程)误差
float B[3] = { 0.259747,0.144082,-0.613522};												//默认零位误差
*/
Sensor_Health Sensor_Flag;
Vector3f accel_out;
static uint8_t raw_count=0;
float tf_buffer[2][filter_number];
float xyz_buffer[3][filter_number];
float tf_data_buffer[2][N2];
//快速倒开方
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

volatile float Sin_Pitch = 0, Sin_Roll = 0, Sin_Yaw = 0;
volatile float Cos_Pitch = 0, Cos_Roll = 0, Cos_Yaw = 0;
volatile float rMat[3][3];
float sqf(float x) {
	return ((x) * (x));
}
void imuComputeRotationMatrix(void) {
	Sin_Pitch = sin(Pitch * DEG2RAD);
	Cos_Pitch = cos(Pitch * DEG2RAD);
	Sin_Roll = sin(Roll * DEG2RAD);
	Cos_Roll = cos(Roll * DEG2RAD);
	Sin_Yaw = sin(Yaw * DEG2RAD);
	Cos_Yaw = cos(Yaw * DEG2RAD);

	Inclination_Rate = (rMat[2][2] - Cos_Pitch * Cos_Roll);

	rMat[0][0] = Cos_Yaw * Cos_Roll;
	rMat[0][1] = Sin_Pitch * Sin_Roll * Cos_Yaw - Cos_Pitch * Sin_Yaw;
	rMat[0][2] = Sin_Pitch * Sin_Yaw + Cos_Pitch * Sin_Roll * Cos_Yaw;

	rMat[1][0] = Sin_Yaw * Cos_Roll;
	rMat[1][1] = Sin_Pitch * Sin_Roll * Sin_Yaw + Cos_Pitch * Cos_Yaw;
	rMat[1][2] = Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;

	rMat[2][0] = -Sin_Roll;
	rMat[2][1] = Sin_Pitch * Cos_Roll;
	rMat[2][2] = Cos_Pitch * Cos_Roll;

}



float Accel_For_Cal[3] = { 0 };
Vector3f gyro, accel;
Vector3f gyro_filter, accel_filter, mag_filter, gyro_filter_bug;
Vector3f gyro_bpf_filter, accel_bpf_filter, accel_bpf_filter_bug;
Vector3f ins_accel_filter;
Butter_BufferData gyro_filter_buf_bug[3],
		gyro_filter_buf[3], accel_filter_buf[3],tf_fliter_data[2],
		accel_cal_filter_buf[3], accel_for_cal_filter_buf[3];
Butter_BufferData gyro_bpf_filter_buf[3], accel_bpf_filter_buf[3],
		accel_bpf_filter_buf_bug[3];
void Sensor_Update(void) {
	MPU6050_Read_Data(&IMU_Sensor.gyro_raw, &IMU_Sensor.accel_raw);
	IMU_Sensor.imu_updtate_flag = 1;
	if (Gyro_Safety_Calibration_Flag == 0)
		Gyro_Safety_Calibration_Flag = Gyro_Calibration_Check(
				&IMU_Sensor.gyro_raw);

	//得到校准后的角速度、加速度计数据
	gyro.x = IMU_Sensor.gyro_raw.x - gyro_offset.x;
	gyro.y = IMU_Sensor.gyro_raw.y - gyro_offset.y;
	gyro.z = IMU_Sensor.gyro_raw.z - gyro_offset.z;

	accel.x = K[0] * IMU_Sensor.accel_raw.x - B[0] * One_G_TO_Accel;
	accel.y = K[1] * IMU_Sensor.accel_raw.y - B[1] * One_G_TO_Accel;
	accel.z = K[2] * IMU_Sensor.accel_raw.z - B[2] * One_G_TO_Accel;

/*	accel_out.x=GildeAverageValueFilter_MAG(accel.x,xyz_data[0]);
	accel_out.y=GildeAverageValueFilter_MAG(accel.y,xyz_data[1]);
	accel_out.z=GildeAverageValueFilter_MAG(accel.z,xyz_data[2]);*/
	//原始数据滤波处理
/*	accel_filter.x=biquadFilterApply(&accFilterLPF[0],accel.x);
	accel_filter.y=biquadFilterApply(&accFilterLPF[1],accel.y);
	accel_filter.z=biquadFilterApply(&accFilterLPF[2],accel.z);*/

/*		accel_filter.x=GildeAverageValueFilter_MAG(accel.x,xyz_data_buffer[0]);
		accel_filter.y=GildeAverageValueFilter_MAG(accel.y,xyz_data_buffer[1]);
		accel_filter.z=GildeAverageValueFilter_MAG(accel.z,xyz_data_buffer[2]);*/

	accel_filter.x=	LPButterworth(accel.x, &accel_filter_buf[0],
			&Accel_Parameter);
	accel_filter.y=	LPButterworth(accel.y, &accel_filter_buf[1],
			&Accel_Parameter);
	accel_filter.z=	LPButterworth(accel.z, &accel_filter_buf[2],
			&Accel_Parameter);

	IMU_Sensor.acce_filter[0]=biquadFilterApply(&accFilterLPF[3],IMU_Sensor.accel_raw.x);
	IMU_Sensor.acce_filter[1]=biquadFilterApply(&accFilterLPF[4],IMU_Sensor.accel_raw.y);
	IMU_Sensor.acce_filter[2]=biquadFilterApply(&accFilterLPF[5],IMU_Sensor.accel_raw.z);

	//Pitch_t=atan2f(-accel_filter.x,sqrt(accel_filter.y*accel_filter.y+accel_filter.z*accel_filter.z))*RAD2DEG;
	//Pitch_t=atan2f(-accel_out.x,sqrt(accel_out.y*accel_out.y+accel_out.z*accel_out.z))*RAD2DEG;
	//Pitch_t=atan2f(-accel.x,sqrt(accel.y*accel.y+accel.z*accel.z))*RAD2DEG;
	//SIN_PITCH=sinf(Pitch_t*DEG2RAD);
	//IMUupdate(gyro_filter.x,gyro_filter.y,gyro_filter.z,accel_filter.x,accel_filter.y,accel_filter.z);
	//IMUupdate(gyro.x,gyro.y,gyro.z,accel.x,accel.y,accel.z);
}

//void Observation_Angle_Calculate() {
//	float ACCE_X_TEMP, ACCE_Y_TEMP, ACCE_Z_TEMP;
//	float MagTemp[3] = { 0 };
//	ACCE_X_TEMP = accel.x;
//	ACCE_Y_TEMP = accel.y;
//	ACCE_Z_TEMP = accel.z;
//	Roll_Observation = -57.3
//			* atan(
//					ACCE_X_TEMP
//							* invSqrt(
//									ACCE_Y_TEMP * ACCE_Y_TEMP
//											+ ACCE_Z_TEMP * ACCE_Z_TEMP));//横滚角
//	Pitch_Observation = 57.3
//			* atan(
//					ACCE_Y_TEMP
//							* invSqrt(
//									ACCE_X_TEMP * ACCE_X_TEMP
//											+ ACCE_Z_TEMP * ACCE_Z_TEMP));//俯仰角
//											/************磁力计倾角补偿*****************/
//	MagTemp[0] = mag_filter.x;
//	MagTemp[1] = mag_filter.y;
//	MagTemp[2] = mag_filter.z;
//
//	magn.x = MagTemp[0] * Cos_Roll + MagTemp[2] * Sin_Roll;
//	magn.y = MagTemp[0] * Sin_Pitch * Sin_Roll + MagTemp[1] * Cos_Pitch
//			- MagTemp[2] * Sin_Pitch * Cos_Roll;
//	/***********反正切得到磁力计观测角度*********/
//	Yaw_Observation = atan2(magn.x, magn.y) * 57.296f;
//}
//
//float euler_roll_init, euler_pitch_init, euler_yaw_init;
//float q0_init, q1_init, q2_init, q3_init;
//
//void Euler_Angle_Init(void) {
//	float ACCE_X_TEMP, ACCE_Y_TEMP, ACCE_Z_TEMP;
//
//	MPU6050_Read_Data(&IMU_Sensor.gyro_raw, &IMU_Sensor.accel_raw);
//	accel.x = K[0] * IMU_Sensor.accel_raw.x - B[0] * One_G_TO_Accel;
//	accel.y = K[1] * IMU_Sensor.accel_raw.y - B[1] * One_G_TO_Accel;
//	accel.z = K[2] * IMU_Sensor.accel_raw.z - B[2] * One_G_TO_Accel;
//	ACCE_X_TEMP = accel.x;
//	ACCE_Y_TEMP = accel.y;
//	ACCE_Z_TEMP = accel.z;
//	Roll_Observation = -57.3 * atan(ACCE_X_TEMP * invSqrt(ACCE_Y_TEMP * ACCE_Y_TEMP + ACCE_Z_TEMP * ACCE_Z_TEMP));//横滚角
//	Pitch_Observation = 57.3 * atan(ACCE_Y_TEMP * invSqrt(ACCE_X_TEMP * ACCE_X_TEMP + ACCE_Z_TEMP * ACCE_Z_TEMP));//俯仰角
//
//	Sin_Pitch = sin(Pitch_Observation * DEG2RAD);
//	Cos_Pitch = cos(Pitch_Observation * DEG2RAD);
//	Sin_Roll = sin(Roll_Observation * DEG2RAD);
//	Cos_Roll = cos(Roll_Observation * DEG2RAD);

//	/************磁力计倾角补偿*****************/
//	MagTemp[0] = IMU_Sensor.mag_raw.x - mag_offset.x;
//	MagTemp[1] = IMU_Sensor.mag_raw.y - mag_offset.y;
//	MagTemp[2] = IMU_Sensor.mag_raw.z - mag_offset.z;
//
//	magn.x = MagTemp[0] * Cos_Roll + MagTemp[2] * Sin_Roll;
//	magn.y = MagTemp[0] * Sin_Pitch * Sin_Roll + MagTemp[1] * Cos_Pitch
//			- MagTemp[2] * Sin_Pitch * Cos_Roll;
//
//	/***********反正切得到磁力计观测角度*********/
//	Yaw_Observation = atan2(magn.x, magn.y) * 57.296f;
//	euler_pitch_init = Roll_Observation * DEG2RAD; //初始化欧拉翻滚角
//	euler_roll_init = Pitch_Observation * DEG2RAD; //初始化欧拉俯仰角
//	euler_yaw_init = Yaw_Observation * DEG2RAD;
//}

//void Euler_Angle_Init_Again(void) {
//	float ACCE_X_TEMP, ACCE_Y_TEMP, ACCE_Z_TEMP;
//	float MagTemp[3] = { 0 };
//	accel.x = K[0] * IMU_Sensor.accel_raw.x - B[0] * One_G_TO_Accel;
//	accel.y = K[1] * IMU_Sensor.accel_raw.y - B[1] * One_G_TO_Accel;
//	accel.z = K[2] * IMU_Sensor.accel_raw.z - B[2] * One_G_TO_Accel;
//	ACCE_X_TEMP = accel.x;
//	ACCE_Y_TEMP = accel.y;
//	ACCE_Z_TEMP = accel.z;
//	Roll_Observation = -57.3
//			* atan(ACCE_X_TEMP * invSqrt(ACCE_Y_TEMP * ACCE_Y_TEMP + ACCE_Z_TEMP * ACCE_Z_TEMP)); //横滚角
//	Pitch_Observation = 57.3
//			* atan(ACCE_Y_TEMP * invSqrt(ACCE_X_TEMP * ACCE_X_TEMP + ACCE_Z_TEMP * ACCE_Z_TEMP)); //俯仰角
//	Sin_Pitch = sin(Pitch_Observation * DEG2RAD);
//	Cos_Pitch = cos(Pitch_Observation * DEG2RAD);
//	Sin_Roll = sin(Roll_Observation * DEG2RAD);
//	Cos_Roll = cos(Roll_Observation * DEG2RAD);
//	/************磁力计倾角补偿*****************/
//	MagTemp[0] = IMU_Sensor.mag_raw.x - mag_offset.x;
//	MagTemp[1] = IMU_Sensor.mag_raw.y - mag_offset.y;
//	MagTemp[2] = IMU_Sensor.mag_raw.z - mag_offset.z;
//	magn.x = MagTemp[0] * Cos_Roll + MagTemp[2] * Sin_Roll;
//	magn.y = MagTemp[0] * Sin_Pitch * Sin_Roll + MagTemp[1] * Cos_Pitch
//			- MagTemp[2] * Sin_Pitch * Cos_Roll;
//	/***********反正切得到磁力计观测角度*********/
//	Yaw_Observation = atan2(magn.x, magn.y) * 57.296f;
//	euler_pitch_init = Roll_Observation * DEG2RAD; //初始化欧拉翻滚角
//	euler_roll_init = Pitch_Observation * DEG2RAD; //初始化欧拉俯仰角
//	euler_yaw_init = Yaw_Observation * DEG2RAD;
//	q0 = cos(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			+ sin(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//	q1 = cos(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* sin(euler_roll_init / 2)
//			- sin(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* cos(euler_roll_init / 2);
//	q2 = cos(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			+ sin(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//	q3 = sin(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			- cos(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//}
//
///****************** 根据初始化欧拉角初始化四元数 *****************************/
//void Quaternion_Init() {
//	Euler_Angle_Init();
//	q0_init = cos(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			+ sin(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//	q1_init = cos(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* sin(euler_roll_init / 2)
//			- sin(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* cos(euler_roll_init / 2);
//	q2_init = cos(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			+ sin(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//	q3_init = sin(euler_yaw_init / 2) * cos(euler_pitch_init / 2)
//			* cos(euler_roll_init / 2)
//			- cos(euler_yaw_init / 2) * sin(euler_pitch_init / 2)
//					* sin(euler_roll_init / 2);
//}
//
//void WP_Quad_Init(void) //初始四元数初始化
//{
//	Quaternion_Init();
//	q0 = q0_init;
//	q1 = q1_init;
//	q2 = q2_init;
//	q3 = q3_init;
//}
//Testime Imu_Delta;
//volatile float IMU_Dt = 0.0f;
//float Yaw_Gyro_Earth_Frame = 0;
//#define sampleFreq 200
//float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;
//volatile float beta = 0.0075f; //0.01
//float gx_delta = 0, gy_delta = 0, gz_delta = 0;
//float kp = 0;
//void Madgwick_AHRS_Update_IMU(float gx, float gy, float gz, float ax, float ay,
//		float az, float mx, float my, float mz, float gyro_mold) {
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float qDot1, qDot2, qDot3, qDot4;
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
//	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//	float hx, hy, bx, bz;
//	float halfwx, halfwy, halfwz;
//	float halfex, halfey, halfez;
//	float beta_temp = 0;
//	Test_Period(&Imu_Delta);
//	IMU_Dt = (float) (Imu_Delta.Time_Delta / 1000.0f);
//	if (IMU_Dt > 2 * WP_Duty_Dt || IMU_Dt < WP_Duty_Dt || isnan(IMU_Dt) != 0)
//		IMU_Dt = WP_Duty_Dt;
//	//IMU_Dt=0.0025f;
//	gx *= GYRO_CALIBRATION_COFF;
//	gy *= GYRO_CALIBRATION_COFF;
//	gz *= GYRO_CALIBRATION_COFF;
//	//{-sinθ          cosθsin Φ                          cosθcosΦ                   }
//	Yaw_Gyro_Earth_Frame = -Sin_Roll * gx + Cos_Roll * Sin_Pitch * gy
//			+ Cos_Pitch * Cos_Roll * gz;
//
//	gx += gx_delta;
//	gy += gy_delta;
//	gz += gz_delta;
//
//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * DEG2RAD;
//	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) * DEG2RAD;
//	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) * DEG2RAD;
//	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) * DEG2RAD;
//
//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//		// Normalise accelerometer measurement
//		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
//		ax *= recipNorm;
//		ay *= recipNorm;
//		az *= recipNorm;
//		// Normalise magnetometer measurement
//		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
//		mx *= recipNorm;
//		my *= recipNorm;
//		mz *= recipNorm;
//
//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0 = 2.0f * q0;
//		_2q1 = 2.0f * q1;
//		_2q2 = 2.0f * q2;
//		_2q3 = 2.0f * q3;
//		_4q0 = 4.0f * q0;
//		_4q1 = 4.0f * q1;
//		_4q2 = 4.0f * q2;
//		_8q1 = 8.0f * q1;
//		_8q2 = 8.0f * q2;
//		q0q0 = q0 * q0;
//		q0q1 = q0 * q1;
//		q0q2 = q0 * q2;
//		q0q3 = q0 * q3;
//		q1q1 = q1 * q1;
//		q1q2 = q1 * q2;
//		q1q3 = q1 * q3;
//		q2q2 = q2 * q2;
//		q2q3 = q2 * q3;
//		q3q3 = q3 * q3;
//
//		// Reference direction of Earth's magnetic field
//		hx = 2.0f
//				* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
//						+ mz * (q1q3 + q0q2));
//		hy = 2.0f
//				* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
//						+ mz * (q2q3 - q0q1));
//		bx = sqrt(hx * hx + hy * hy);
//		bz = 2.0f
//				* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
//						+ mz * (0.5f - q1q1 - q2q2));
//
//		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
//		// Error is sum of cross product between estimated direction and measured direction of field vectors
//		halfex = (my * halfwz - mz * halfwy);
//		halfey = (mz * halfwx - mx * halfwz);
//		halfez = (mx * halfwy - my * halfwx);
//
//		gx_delta = kp * halfex;
//		gy_delta = kp * halfey;
//		gz_delta = kp * halfez;
//
//		// Gradient decent algorithm corrective step
//		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1
//				+ _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
//				+ _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
//		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;
//
//		beta_temp = beta + 0.025f * IMU_Dt * constrain_float(gyro_mold, 0, 500);
//		beta_temp = constrain_float(beta_temp, beta, 0.06f);
//
//		// Apply feedback step
//		qDot1 -= beta_temp * s0;
//		qDot2 -= beta_temp * s1;
//		qDot3 -= beta_temp * s2;
//		qDot4 -= beta_temp * s3;
//	}
//
//	// Integrate rate of change of quaternion to yield quaternion
//	q0 += qDot1 * IMU_Dt;
//	q1 += qDot2 * IMU_Dt;
//	q2 += qDot3 * IMU_Dt;
//	q3 += qDot4 * IMU_Dt;
//
//	// Normalise quaternion
//	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//}
#if 0
float Yaw_Temp = 0;
Testime T1, T2;
float Sensor_Time = 0;
void Get_Status_Feedback(void) {
	Test_Period(&T1); //更新姿态数据
	Sensor_Update(); //传感器数据更新
	Test_Period(&T2);
	Sensor_Time = T2.Now_Time - T1.Now_Time;
	Observation_Angle_Calculate(); //观测量计算
	Madgwick_AHRS_Update_IMU(gyro_filter.x, gyro_filter.y, gyro_filter.z,
			accel_filter.x, accel_filter.y, accel_filter.z, mag_filter.x,
			mag_filter.y, mag_filter.z, Gyro_Length);
	Pitch = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1,
			-2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * RAD2DEG;		// Pitch
	Roll = asin(2.0f * q0 * q2 - 2.0f * q1 * q3) * RAD2DEG;				// Roll

#if (YAW_FUSION_MODE_QUAD)
	Yaw = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3,
			-2.0f * q3 * q3 - 2.0f * q2 * q2 + 1.0f) * RAD2DEG;		// Yaw
#else
			//偏航角一阶互补融合
			Yaw_Temp+=Yaw_Gyro_Earth_Frame*IMU_Dt;
			if((Yaw_Observation>90 && Yaw_Temp<-90)
					|| (Yaw_Observation<-90 && Yaw_Temp>90))
			Yaw_Temp = -Yaw_Temp * 0.99f + Yaw_Observation * 0.01f;
			else Yaw_Temp = Yaw_Temp * 0.99f + Yaw_Observation * 0.01f;
			//偏航角数据量化到0~360
			if(Yaw_Temp<0) Yaw=Yaw_Temp+360;
			else Yaw=Yaw_Temp;
			if(GPS_Home_Set==1) Yaw=Yaw-Declination;//如果GPS home点已设置，获取当地磁偏角，得到地理真北
#endif

	//计算三轴角速度，用于姿态内环控制
	Pitch_Gyro = gyro_filter.x * GYRO_CALIBRATION_COFF;
	Roll_Gyro = gyro_filter.y * GYRO_CALIBRATION_COFF;
	Yaw_Gyro = gyro_filter.z * GYRO_CALIBRATION_COFF;
	Gyro_Length = sqrt(
			Yaw_Gyro * Yaw_Gyro + Pitch_Gyro * Pitch_Gyro
					+ Roll_Gyro * Roll_Gyro);		//单位deg/s
	imuComputeRotationMatrix();

	Circle_Angle.Pit += Pitch_Gyro * IMU_Dt;
	Circle_Angle.Rol += Roll_Gyro * IMU_Dt;
	Circle_Angle.Yaw += Yaw_Gyro * IMU_Dt;
	if (Circle_Angle.Pit < 0.0f)
		Circle_Angle.Pit += 360.0f;
	if (Circle_Angle.Pit > 360.0f)
		Circle_Angle.Pit -= 360.0f;
	if (Circle_Angle.Rol < 0.0f)
		Circle_Angle.Rol += 360.0f;
	if (Circle_Angle.Rol > 360.0f)
		Circle_Angle.Rol -= 360.0f;
	if (Circle_Angle.Yaw < 0.0f)
		Circle_Angle.Yaw += 360.0f;
	if (Circle_Angle.Yaw > 360.0f)
		Circle_Angle.Yaw -= 360.0f;
}
#endif
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
        float norm;
        float vx, vy, vz;
        float ex, ey, ez;
        // 测量正常化
        norm = sqrt(ax*ax + ay*ay + az*az);
        ax = ax / norm;                   //单位化
        ay = ay / norm;
        az = az / norm;
        // 估计方向的重力
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        // 错误的领域和方向传感器测量参考方向之间的交叉乘积的总和
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);
        // 积分误差比例积分增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
        // 调整后的陀螺仪测量
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        // 整合四元数率和正常化
        q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
        // 正常化四元
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
        Pitch_t  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch ,转换为度数
        Roll_t = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // rollv
        //Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                //此处没有价值，注掉
}
void imu_fliter_tf_xyz(serial_t *serial,tfmini_data_t *tf_data,tf_t *tf_filter)
{
	int i=0,j=0,temp_count=0;
	float XYZ_last,XYZ_first,raw_temp_xyz;
	float temp_xyz;
	temp_xyz=10.0;
	Sensor_Update();
	if(raw_count>=0&&raw_count<filter_number)
	{
		if(accFilterLPF[0].fil_flag==false)
		{
			if(TFMini_receive_Data(serial,tf_data)==true)
			{
				tf_buffer[0][raw_count]=LPButterworth((float)tf_data->distance, &tf_fliter_data[0],
						&TF_distance);
				tf_buffer[1][raw_count]=LPButterworth((float)tf_data->strength, &tf_fliter_data[1],
						&TF_strenge);
/*				tf_buffer[0][raw_count]=biquadFilterApply(&accFilterLPF[6],(float)tf_data->distance);
				tf_buffer[1][raw_count]=biquadFilterApply(&accFilterLPF[7],(float)tf_data->strength);
				tf_buffer[0][raw_count]=(float)tf_data->distance;
				tf_buffer[1][raw_count]=(float)tf_data->strength;*/
			}
		}
			xyz_buffer[0][raw_count]=accel_filter.x;
			xyz_buffer[1][raw_count]=accel_filter.y;
			xyz_buffer[2][raw_count]=accel_filter.z;
		raw_count++;
	}
	if(raw_count==filter_number)
	{
		j=0;
		while(j<13)
		{
			XYZ_last=sqrt(xyz_buffer[0][j+6]*xyz_buffer[0][j+6]+xyz_buffer[1][j+6]*xyz_buffer[1][j+6]+xyz_buffer[2][j+6]*xyz_buffer[2][j+6]);
			XYZ_first=sqrt(xyz_buffer[0][j]*xyz_buffer[0][j]+xyz_buffer[1][j]*xyz_buffer[1][j]+xyz_buffer[2][j]*xyz_buffer[2][j]);
			raw_temp_xyz=fabs(XYZ_last-XYZ_first);
			if(raw_temp_xyz<10)
			if(temp_xyz>raw_temp_xyz)
			{
				temp_xyz=raw_temp_xyz;
				temp_count=j;
			}
			j++;
		}
		for(i=0;i<7;i++)
		{
			accel_out.x+=xyz_buffer[0][i+temp_count]/7;
			accel_out.y+=xyz_buffer[1][i+temp_count]/7;
			accel_out.z+=xyz_buffer[2][i+temp_count]/7;
/*			tf_filter->distance+=GildeAverageValueFilter_MAG(tf_buffer[0][i+temp_count],tf_data_buffer[0])/7;
			tf_filter->strength+=GildeAverageValueFilter_MAG(tf_buffer[1][i+temp_count],tf_data_buffer[1])/7;
			accel_out.x+=GildeAverageValueFilter_MAG(xyz_buffer[0][i+temp_count],xyz_data_buffer[0])/7;
			accel_out.y+=GildeAverageValueFilter_MAG(xyz_buffer[1][i+temp_count],xyz_data_buffer[1])/7;
			accel_out.z+=GildeAverageValueFilter_MAG(xyz_buffer[2][i+temp_count],xyz_data_buffer[2])/7;*/
		}
		tf_filter->distance=tf_buffer[0][temp_count];
		tf_filter->strength=tf_buffer[1][temp_count];
/*		tf_filter->distance=GildeAverageValueFilter_MAG(tf_buffer[0][temp_count],tf_data_buffer[0]);
		tf_filter->strength=GildeAverageValueFilter_MAG(tf_buffer[1][temp_count],tf_data_buffer[1]);
		accel_out.x=GildeAverageValueFilter_MAG(xyz_buffer[0][temp_count],xyz_data_buffer[0]);
		accel_out.y=GildeAverageValueFilter_MAG(xyz_buffer[1][temp_count],xyz_data_buffer[1]);
		accel_out.z=GildeAverageValueFilter_MAG(xyz_buffer[2][temp_count],xyz_data_buffer[2]);*/
		accFilterLPF[0].fil_flag=true;
		raw_count=0;
		memset(tf_buffer,0,2*filter_number);
		memset(xyz_buffer,0,3*filter_number);
	}
}
void text_filter(serial_t *serial,tfmini_data_t *tf_data,tf_t *tf_filter)
{
	Sensor_Update();
	if(TFMini_receive_Data(serial,tf_data)==true)
	{
		tf_filter->distance=LPButterworth((float)tf_data->distance, &tf_fliter_data[0],
				&TF_distance);
		tf_filter->strength=LPButterworth((float)tf_data->strength, &tf_fliter_data[1],
				&TF_strenge);
	}
}
void printf_xyz_tf(serial_t *serial,tfmini_data_t *tf_data,tf_t *tf_filter)
{
	Sensor_Update();
	if(TFMini_receive_Data(serial,tf_data)==true)
	{
		tf_filter->distance=LPButterworth((float)tf_data->distance, &tf_fliter_data[0],
				&TF_distance);
		tf_filter->strength=LPButterworth((float)tf_data->strength, &tf_fliter_data[1],
				&TF_strenge);
	}
	Pitch_t=atan2f(-accel_filter.x,sqrt(accel_filter.y*accel_filter.y+accel_filter.z*accel_filter.z))*RAD2DEG;
	SIN_PITCH=sinf(Pitch_t*DEG2RAD);
	printf("%0.2f  ",accel_filter.x);
	printf("%0.2f  ",accel_filter.y);
	printf("%0.2f  ",accel_filter.z);
	printf("%0.2f  ",tf_filter->distance);
	printf("%0.2f  ",tf_filter->strength);
	printf("%0.2f  ",Pitch_t);
	printf("%0.2f  \r\n",SIN_PITCH*tf_filter->distance);
	tf_filter->distance=tf_filter->strength=0.0;
	accFilterLPF[0].fil_flag=false;
	accel_out.x=accel_out.y=accel_out.z=0.0;
}
