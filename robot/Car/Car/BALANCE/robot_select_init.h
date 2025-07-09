#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//�����˲����ṹ��
typedef struct  
{
  float WheelSpacing;      //�־� ���ֳ�Ϊ���־�
  float AxleSpacing;       //��� ���ֳ�Ϊ�����	
  int GearRatio;           //������ٱ�
  int EncoderAccuracy;     //����������(����������)
  float WheelDiameter;     //������ֱ��	
}Robot_Parament_InitTypeDef;

//�������ṹ��
typedef struct  
{
  int A;      
  int B; 
	int C; 
	int D; 
}Encoder;


//�־�
#define MEC_wheelspacing         0.1535f/2 

//��� 
#define MEC_axlespacing           0.28981f/2

//������ٱ�
#define   MD60N_20    30.0

//����������
#define	  Hall_13           13

//������ֱ̥��
#define		Mecanum_60  0.060f

//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples  4
//���������ݶ�ȡƵ��
#define   CONTROL_FREQUENCY 100

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
