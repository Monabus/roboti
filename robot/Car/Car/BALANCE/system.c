#include "system.h"

//���������ʧ�ܱ�־λ
u8 Flag_Stop=1;   

//ADCֵ�ֶα�����ȡ����С���ͺ�����
int Divisor_Mode;

//�������ͺű���
u8 Car_Mode=0; 

int Servo;  

//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=500; 

//С��ˢ�ӿ��Ʊ�־λ
int Brush_Flag = 0;
int Step_Num = 0;

//����������
int Sprayer_Flag = 0;

//��������������ź�
int Guangdian_Flag = 0;

//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z;   

//�ٶȿ���PID����
float Velocity_KP=300,Velocity_KI=300; 

//ƽ�������м������ȫ���ƶ�С��ר��
Smooth_Control smooth_control;  

//����Ĳ����ṹ��
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ С����ر��� **************************/
//����������
float Encoder_precision; 
//�����ܳ�����λ��m
float Wheel_perimeter; 
//�������־࣬��λ��m
float Wheel_spacing; 
//С��ǰ�������࣬��λ��m
float Axle_spacing; 
/************ С��������ر��� **************************/

//����APP������1������5ͨ�ſ��Ʊ�־λ ������3����ģʽ
u8  APP_ON_Flag=0, Usart1_ON_Flag, Usart5_ON_Flag; 

//����ң����صı�־λ
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//����or�Լ�����־λ
u8 Proc_Flag=0; 

//�������ֵ
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
	
	//�ж����ȼ���������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

//	//��ʱ������ʼ��
	delay_init(168);	
	
	//��ʼ����LED�����ӵ�Ӳ���ӿ�
	LED_Init();                     
	    
	//��ʼ����ˢ�����ӵ�Ӳ���ӿ�
	Equipment_init();
	
  //��ʼ������������ӵ�Ӳ���ӿ�
	Buzzer_Init();  
	
	//��ʼ����ʹ�ܿ������ӵ�Ӳ���ӿ�
	Enable_Pin();

  //��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�	
	OLED_Init();     
	
	//��ʼ�����û��������ӵ�Ӳ���ӿ�
	KEY_Init();	

	//����1��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart1_init(115200);	  
	
	//����2��ʼ����ͨ�Ų�����9600������������APP��ͨ��
	uart2_init(9600);  
	
	//����3��ʼ����ͨ�Ų�����115200������3ΪĬ��������ROS��ͨ�ŵĴ���
	uart3_init(115200);
		
	//����4��ʼ����ͨ�Ų�����115200������3Ϊ�������֮���ͨ��
	uart4_init(9600);

	//����5��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
	uart5_init(115200);

	//ADC���ų�ʼ�������ڶ�ȡ��ص�ѹ���λ����λ����λ����λ����С���������С�������ͺ�
 	Adc_Init();  
	Adc_POWER_Init();
	
  //������ʼ��	
	Robot_Select();                 
	
  //������A��ʼ�������ڶ�ȡ���C��ʵʱ�ٶ�	
	 Encoder_Init_TIM2();
  //������B��ʼ�������ڶ�ȡ���D��ʵʱ�ٶ�	
	  Encoder_Init_TIM3();   
  //������C��ʼ�������ڶ�ȡ���B��ʵʱ�ٶ�	
	  Encoder_Init_TIM4(); 
	//������D��ʼ�������ڶ�ȡ���A��ʵʱ�ٶ�
		Encoder_Init_TIM5(); 
	
	//��ʱ��8��ΪBrush�ٶȿ��ƽӿ�
		TIM8_SERVO_Init(16799,0);
		
	//��ʱ��12����������PWM�ӿ�
		TIM12_SERVO_Init(299,84-1);  //APB1��ʱ��Ƶ��Ϊ84M , Ƶ��=84M/((499+1)*(83+1))=2000Hz
   

  //��ʼ������ٶȿ����Լ������ڿ��Ƶ���ٶȣ�PWMƵ��10KHZ
  //APB2ʱ��Ƶ��Ϊ168M����PWMΪ16799��Ƶ��=168M/((16799+1)*(0+1))=10k
		TIM1_PWM_Init(16799,0);
		TIM9_PWM_Init(16799,0);
		TIM10_PWM_Init(16799,0);
		TIM11_PWM_Init(16799,0);
		
  //IIC��ʼ��������MPU6050  
	I2C_GPIOInit();

  //MPU6050 ��ʼ�������ڶ�ȡС��������̬��������ٶȡ�������ٶ���Ϣ
   MPU6050_initialize();        		
			 							
}
