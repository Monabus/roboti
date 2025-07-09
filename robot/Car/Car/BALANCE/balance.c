#include "balance.h"

int Time_count=0;  //��ʱ����
u32 Buzzer_count1 = 0; 
int Step_Flag = 0;
short test_num;
int Down_flag = 0;	
Encoder OriginalEncoder;  //������ԭʼ����
int Last_Brush_Flag = 0;  //��ֹ�����ť��

u8 command_lost_count=0; //���ڿ������ʧʱ���������ʧ1���ֹͣ����
/**************************************************************************
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
		Smooth_control(Vx,Vy,Vz); //�������ٶȽ���ƽ������
  
		//��ȡƽ������������			
		Vx=smooth_control.VX;     
		Vy=smooth_control.VY;
		Vz=smooth_control.VZ;
		
		//�˶�ѧ���
		MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
		//����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
}
/**************************************************************************
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
			//��������100Hz��Ƶ�����У�10ms����һ�Σ�
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			
			//ʱ�������30�������Ҫ
			if(Time_count<3000)Time_count++;
			Buzzer_count1++;
			//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
			Get_Velocity_Form_Encoder();   
			//����User�����������������
			Key();

			if (APP_ON_Flag)   
				Get_RC();          							//����APPң������
			else     
				Drive_Motor(Move_X, Move_Y, Move_Z);		//����3(ROS)����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
			 
			Brush_Control();
			
			//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0
			if(Turn_Off(Voltage)==0) 
			 { 							 
				 //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��
				 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
						 
				 Limit_Pwm(16700);
				 
				 Set_Pwm( MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Step_Flag); 
			 }
			 //���Turn_Off(Voltage)����ֵΪ1�����������С�������˶���PWMֵ����Ϊ0
			 else	Set_Pwm(0,0,0,0,0);
	 }
}
/**************************************************************************
�������ܣ���־λ�����źţ����������½�
��ڲ������ź�
����  ֵ����
**************************************************************************/
void Brush_Control( )
{	
			if(Last_Brush_Flag != Brush_Flag && Brush_Flag != 0)
			{
				Last_Brush_Flag =  Brush_Flag;
				Brush_Flag = 0;
			}
			
			
			if (Brush_Flag == 1) {
					GPIO_ResetBits(GPIOC, GPIO_Pin_1); // ��������Ϊ�͵�ƽ
					Down_flag = 0;
			} else if (Brush_Flag == 6) {
					GPIO_SetBits(GPIOC, GPIO_Pin_1);   // ��������Ϊ�ߵ�ƽ
					Down_flag = 1;										 //�����½�״̬
			}
			
			if(Brush_Flag)
			{
				if(Step_Num < 6700)
				{
					GPIO_ResetBits(GPIOC,GPIO_Pin_0);
					Step_Flag = 50;
				}
				else 
				{
					Brush_Flag = 0;
					Step_Flag = 0;
					Step_Num = 0;
					GPIO_SetBits(GPIOC,GPIO_Pin_0);
				}
			}
			
			//������
			if(Sprayer_Flag == 1)
			{
				GPIO_SetBits(GPIOC,GPIO_Pin_2);					//����
			}
			else if(Sprayer_Flag == 0)
			{
				GPIO_ResetBits(GPIOC,GPIO_Pin_2);				//����
			}
			
			//��ˢ���½���ˢ�ӿ�ʼת��
			if (Down_flag == 1 && Brush_Flag == 0)
			{
				BrushL_PWM = 16700;
				BrushR_PWM = 16700;
			}
			else
			{	
				BrushL_PWM = 0;
				BrushR_PWM = 0;
			}		
		
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//�������ת����
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//�������ת����	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//�������ת����	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//�������ת����
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//�������
	Servo_PWM =servo;
}

/**************************************************************************
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC2=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
/**************************************************************************
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;

	 switch(Flag_Direction)  //�������������
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500);  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500);  //����ת
		 else 		               Move_Z=0;                        //ֹͣ
	 }

	  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/*************************************************************************
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 

	if(tmp==1)		memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));

}
/**************************************************************************
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	
	
		//������������ֵ����
		Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D;
	
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

