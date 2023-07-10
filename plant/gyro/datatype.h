#ifndef DATATYPE_H_
#define DATATYPE_H_

#include "math.h"
#include "MY_Math.h"

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

#define int16  short
#define uint16 unsigned short
#define int32  int
#define uint32 unsigned int
#define uint8  unsigned char
#define s32    int32

#define Int_Sort (int16_t)
#define TRUE  true
#define FALSE false

//typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} vector3i;

typedef struct {
	float Last_Time;
	float Now_Time;
	float Time_Delta;
	uint16_t Time_Delta_INT; //单位ms
} Testime;

typedef struct {
	int32_t x;
	int32_t y;
} Vector2i;

typedef struct {
	float x;
	float y;
} Vector2f;

typedef struct {
	float x;
	float y;
	float z;
} Vector3f;

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
} Vector4q;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3i;

typedef struct {
	float E;
	float N;
	float U;
} Vector3_Nav;

typedef struct {
	float E;
	float N;
} Vector2f_Nav;

typedef struct {
	int32_t lat;
	int32_t lng;
} Vector2_Nav;

typedef struct {
	float x;
	float y;
	float z;
} Vector3_Body;

typedef struct {
	float Pit;
	float Rol;
} Vector2_Ang;

typedef struct {
	float Pit;
	float Rol;
} Vector2_Body;

typedef struct {
	float Pit;
	float Rol;
	float Yaw;
} Vector3f_Body;

typedef struct {
	float North;
	float East;
} Vector2_Earth;

typedef struct {
	Vector3f a;
	Vector3f b;
	Vector3f c;
} Matrix3f;

typedef struct {
	float max;
	float min;
	float middle;
	float deadband;
} Vector_RC;

typedef struct {
	Vector3f gyro_raw;
	Vector3f accel_raw;
	float acce_filter[3]; //原始数据滤波，用于校准
	uint8_t imu_updtate_flag;

} Sensor;

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;


typedef struct
{
 float Output_Butter[3];
}Notch_Filter_BufferData;


typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;


#define vector3f Vector3f


typedef struct {
	uint8_t Mpu_Health;
	uint8_t Baro_Health;
	uint8_t Mag_Health;
	uint8_t Gps_Health;
	uint8_t Hcsr04_Health;
} Sensor_Health;

#endif /* DATATYPE_H_ */
