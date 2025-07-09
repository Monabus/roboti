#include "robot_select_init.h"

//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
  //ADC值分段变量
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode);  //采集电位器引脚信息	
  if(Car_Mode>5)Car_Mode=5;

	Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     MD60N_20, Hall_13, Mecanum_60);

}

/**************************************************************************
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//轮距 麦轮车为半轮距
  Robot_Parament.WheelSpacing=wheelspacing; 
  //轴距 麦轮车为半轴距	
  Robot_Parament.AxleSpacing=axlespacing;   

	//电机减速比
  Robot_Parament.GearRatio=gearratio; 
  //编码器精度(编码器线数)	
  Robot_Parament.EncoderAccuracy=Accuracy;
  //主动轮直径	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
  //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
  //轮距 麦轮车为半轮距  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //轴距 麦轮车为半轴距	
	Axle_spacing=Robot_Parament.AxleSpacing; 
}


