#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //轮距 麦轮车为半轮距
  float AxleSpacing;       //轴距 麦轮车为半轴距	
  int GearRatio;           //电机减速比
  int EncoderAccuracy;     //编码器精度(编码器线数)
  float WheelDiameter;     //主动轮直径	
}Robot_Parament_InitTypeDef;

//编码器结构体
typedef struct  
{
  int A;      
  int B; 
	int C; 
	int D; 
}Encoder;


//轮距
#define MEC_wheelspacing         0.1535f/2 

//轴距 
#define MEC_axlespacing           0.28981f/2

//电机减速比
#define   MD60N_20    30.0

//编码器精度
#define	  Hall_13           13

//麦轮轮胎直径
#define		Mecanum_60  0.060f

//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples  4
//编码器数据读取频率
#define   CONTROL_FREQUENCY 100

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
