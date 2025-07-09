#include "robot_select_init.h"

//��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
  //ADCֵ�ֶα���
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode);  //�ɼ���λ��������Ϣ	
  if(Car_Mode>5)Car_Mode=5;

	Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     MD60N_20, Hall_13, Mecanum_60);

}

/**************************************************************************
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//�־� ���ֳ�Ϊ���־�
  Robot_Parament.WheelSpacing=wheelspacing; 
  //��� ���ֳ�Ϊ�����	
  Robot_Parament.AxleSpacing=axlespacing;   

	//������ٱ�
  Robot_Parament.GearRatio=gearratio; 
  //����������(����������)	
  Robot_Parament.EncoderAccuracy=Accuracy;
  //������ֱ��	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	//���(����)ת1Ȧ��Ӧ�ı�������ֵ
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
  //�������ܳ�	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
  //�־� ���ֳ�Ϊ���־�  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //��� ���ֳ�Ϊ�����	
	Axle_spacing=Robot_Parament.AxleSpacing; 
}


