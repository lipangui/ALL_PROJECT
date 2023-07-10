#ifndef __FILTER_H
#define __FILTER_H
#include "imu.h"
#include "calibration.h"
#define BIQUAD_Q		1.0f/sqrt(2.0f)
#define N2 9
 typedef struct
 {
      float b0, b1, b2, a1, a2;
      float d1, d2;
      bool fil_flag;
  } biquadFilter_t;
//软件二阶低通滤波参数（单位Hz）
#define ACCEL_LPF_CUTOFF_FREQ  10.0f
extern biquadFilter_t accFilterLPF[8];


void FilterInit(float accUpdateRate);
float biquadFilterApply(biquadFilter_t *filter,float input);
void biquadFilterInit(biquadFilter_t *filter,uint16_t samplingFreq, uint16_t filterFreq, float Q);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
float GildeAverageValueFilter_MAG(float NewValue,float *Data);
//巴特沃斯滤波器参数
typedef struct
{
    int N;          //巴特沃斯滤波器最小实现阶数
    int length;     //滤波器系统函数系数数组的长度
    float fc;       //巴特沃斯滤波器截止频率
    float cosW0;    //中心频率，带通带阻时用到
    float fs;       //采样频率
    int filterType; //需要设计的数字滤波器类型
	  float pass[2];
	  float stop[2];
	  float rp ;//通带衰减，典型值2dB
	  float rs ;//阻带衰减，典型值20dB
	  float sbvalue[11];
	  float num[11];
	  float den[11];
	  float input[11];
	  float output[11];
	  bool isFOK;
}ButterFilterStruct;



extern Butter_Parameter Bandstop_Filter_Parameter_30_98,Bandstop_Filter_Parameter_30_94;

extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;


float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Acce_Control_Filter(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
void Butterworth_Parameter_Init(void);
void Test_Filter(void);
float BPF_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);


void pascalTriangle(int N,int symbol,int *vector);
bool filter(ButterFilterStruct butterValue,float *numerator,float *denominator,float *xVector,int length,float *yVector);
bool BTfilter(ButterFilterStruct *butterValue,float data);

extern ButterFilterStruct Gyro_X_Butter_Filter,Gyro_Y_Butter_Filter,Gyro_Z_Butter_Filter;
extern ButterFilterStruct Accel_X_Butter_Filter,Accel_Y_Butter_Filter,Accel_Z_Butter_Filter;
extern Butter_Parameter Bandstop_Filter_Parameter_30_98;
extern Butter_Parameter Bandstop_Filter_Parameter_30_94;


#endif
