#ifndef __SYSTEM_H
#define __SYSTEM_H

//����������Ҫ�õ���ͷ�ļ�
#include "FreeRTOSConfig.h"
//FreeRTOS���ͷ�ļ� 
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//��������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "key.h"
#include "robot_select_init.h"
#include "I2C.h"
#include "MPU6050.h"


//����ٶȿ�����ز����ṹ��
typedef struct  
{
	float Encoder;     //��������ֵ����ȡ���ʵʱ�ٶ�
	float Motor_Pwm;   //���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target;      //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity_KP; //�ٶȿ���PID����
	float	Velocity_KI; //�ٶȿ���PID����
}Motor_parameter;

//ƽ�������������ٶ�
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8  APP_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern int Brush_Flag, Step_Num, Sprayer_Flag, Guangdian_Flag; 

extern u8 Proc_Flag;
extern int servo_direction[6],servo_pwm_count;
extern int check_time_count_motor_forward,check_time_count_motor_retreat;
extern int POT_val;
extern int Servo_Count[6];
extern u8 uart3_receive_message[50];
extern u8 uart3_send_flag;
extern u8 message_count;
extern u8 uart2_send_flag;
extern u8 app_count;
extern int Full_rotation;

void systemInit(void);
/***�궨��***/
//����(1000/100hz=10)�����������С�������˶�
#define CONTROL_DELAY		1000
//�������ͺ�����������Divisor_Mode��ֵ��Ŀǰ��6��С������
#define CAR_NUMBER    6      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***�궨��***/

//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif 
