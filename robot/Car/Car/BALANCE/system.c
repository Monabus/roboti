#include "system.h"

//机器人软件失能标志位
u8 Flag_Stop=1;   

//ADC值分段变量，取决于小车型号数量
int Divisor_Mode;

//机器人型号变量
u8 Car_Mode=0; 

int Servo;  

//遥控小车的默认速度，单位：mm/s
float RC_Velocity=500; 

//小车刷子控制标志位
int Brush_Flag = 0;
int Step_Num = 0;

//喷雾器开关
int Sprayer_Flag = 0;

//光电检测垃圾满载信号
int Guangdian_Flag = 0;

//小车三轴目标运动速度，单位：m/s
float Move_X, Move_Y, Move_Z;   

//速度控制PID参数
float Velocity_KP=300,Velocity_KI=300; 

//平滑控制中间变量，全向移动小车专用
Smooth_Control smooth_control;  

//电机的参数结构体
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ 小车相关变量 **************************/
//编码器精度
float Encoder_precision; 
//轮子周长，单位：m
float Wheel_perimeter; 
//主动轮轮距，单位：m
float Wheel_spacing; 
//小车前后轴的轴距，单位：m
float Axle_spacing; 
/************ 小车控制相关变量 **************************/

//蓝牙APP、串口1、串口5通信控制标志位 代表串口3控制模式
u8  APP_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag; 

//蓝牙遥控相关的标志位
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//控制or自检代码标志位
u8 Proc_Flag=0; 

//舵机计数值
int servo_direction[6] = {0};
int servo_pwm_count = 500;

int check_time_count_motor_forward=300,check_time_count_motor_retreat=500;
int POT_val;

int Servo_Count[6] = {500, 500, 500, 500, 500, 500};

u8 uart3_receive_message[50];
u8 uart3_send_flag;
u8 message_count=0;

u8 uart2_receive_message[50];
u8 uart2_send_flag=5;
u8 app_count=0;

int Full_rotation = 16799;

void systemInit(void)
{       
	
	//中断优先级分组设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	//延时函数初始化
	delay_init(168);	
	
	//初始化与LED灯连接的硬件接口
	LED_Init();                     
	    
	//初始化与刷子连接的硬件接口
	Equipment_init();
	
  //初始化与蜂鸣器连接的硬件接口
	Buzzer_Init();  
	
	//初始化与使能开关连接的硬件接口
	Enable_Pin();

  //初始化与OLED显示屏连接的硬件接口	
	OLED_Init();     
	
	//初始化与用户按键连接的硬件接口
	KEY_Init();	

	//串口1初始化，通信波特率115200，可用于与ROS端通信
	uart1_init(115200);	  
	
	//串口2初始化，通信波特率9600，用于与蓝牙APP端通信
	uart2_init(9600);  
	
	//串口3初始化，通信波特率115200，串口3为默认用于与ROS端通信的串口
	uart3_init(115200);
		
	//串口4初始化，通信波特率115200，串口3为两块板子之间的通信
	uart4_init(9600);

	//串口5初始化，通信波特率115200，可用于与ROS端通信
	uart5_init(115200);

	//ADC引脚初始化，用于读取电池电压与电位器档位，电位器档位决定小车开机后的小车适配型号
 	Adc_Init();  
	Adc_POWER_Init();
	
  //参数初始化	
	Robot_Select();                 
	
  //编码器A初始化，用于读取电机C的实时速度	
	 Encoder_Init_TIM2();
  //编码器B初始化，用于读取电机D的实时速度	
	  Encoder_Init_TIM3();   
  //编码器C初始化，用于读取电机B的实时速度	
	  Encoder_Init_TIM4(); 
	//编码器D初始化，用于读取电机A的实时速度
		Encoder_Init_TIM5(); 
	
	//定时器8作为Brush速度控制接口
		TIM8_SERVO_Init(16799,0);
		
	//定时器12用作步进的PWM接口
		TIM12_SERVO_Init(299,84-1);  //APB1的时钟频率为84M , 频率=84M/((499+1)*(83+1))=2000Hz
   

  //初始化电机速度控制以及，用于控制电机速度，PWM频率10KHZ
  //APB2时钟频率为168M，满PWM为16799，频率=168M/((16799+1)*(0+1))=10k
		TIM1_PWM_Init(16799,0);
		TIM9_PWM_Init(16799,0);
		TIM10_PWM_Init(16799,0);
		TIM11_PWM_Init(16799,0);
		
  //IIC初始化，用于MPU6050  
	I2C_GPIOInit();

  //MPU6050 初始化，用于读取小车三轴姿态、三轴角速度、三轴加速度信息
   MPU6050_initialize();        		
			 							
}
