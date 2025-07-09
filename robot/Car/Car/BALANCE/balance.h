#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		4     //Task priority //�������ȼ�
#define BALANCE_STK_SIZE 		512   //Task stack size //�����ջ��С

extern short test_num;
extern u8 command_lost_count; //���ڿ������ʧʱ���������ʧ1���ֹͣ����
void Balance_task(void *pvParameters);
void Brush_Control(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Get_RC(void);
void Remote_Control(void);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
void PS2_control(void);
float float_abs(float insert);
#endif  

