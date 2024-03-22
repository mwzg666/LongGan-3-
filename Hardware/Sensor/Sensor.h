#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

//===============================变量定义=============================
#if 0
#define USE_LOW_USV 8000		//8mSv
#define USE_HIGH_USV 10000		//10mSv
#endif

//yao 切换临界切换值↓
#define USE_LOW_USV    2000		//2mSv
#define USE_HIGH_USV   1000		//1mSv

//↑切换临界切换值
#define LOW_SEG 0 //低量程段状态
#define HIG_SEG 1 //高量程段状态


//===============================结构体定义===========================




//===============================变量声明=============================

extern u8 GDoseSeg;
extern float CanshuD;

//===============================函数声明=============================
void SensorInit(void);
void SensorMeasureBegin(void);
void CaptureSensorPluseCounter(void);

float Get_Low_Smooth_Counter(void);
float Get_High_Smooth_Counter(void);
float Get_Low_Counter(void);
float Get_High_Counter(void);
void Get_Bendi_Value(char time);
float Netro_Calibration(u32 time,u32 refdata,u8 channel);
u16 CalcAlarmState(LP_SYSTEM_STTAE *me);
#endif