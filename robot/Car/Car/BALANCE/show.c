#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
extern int Time_count;
/**************************************************************************
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count=25;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ)); //��������10Hz��Ƶ������
		
		//����ʱ���������ݷ�������������
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //����������
		else if(Buzzer_count==5)Buzzer=0;
		
		//Read the battery voltage //��ȡ��ص�ѹ
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++;  //ȷ��������ֻ��0.5��
		if(LowVoltage_2==1)LowVoltage_2++;  //ȷ��������ֻ��0.5��
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++;  //10.5V���͵���ʱ��������һ�α���
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++;  //10V��С����ֹ����ʱ�������ڶ��α���
					
		APP_Show();	  //��APP��������
	  oled_show();  //��ʾ����ʾ����
   }
}  

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show;
	
	 //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 POT_val = Get_adc_Average(Potentiometer,10);
	 Car_Mode_Show=(int) (POT_val/Divisor_Mode);
	 if(Car_Mode_Show>5)Car_Mode_Show=5; 
	
	 Voltage_Show=Voltage*100; 
	 count++;
	 memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM���㵫������ˢ�£���ֹ����

		 //��ʾ����1����ʾ����//
//		 OLED_ShowString(0,0,"Mec ");
			
		OLED_ShowNumber(0, 0, Brush_Flag,1,12);

		OLED_ShowNumber(25, 0, Guangdian_Flag,1,12);

		//���֡�ȫ����С����ʾZ����ٶ�
		 OLED_ShowString(55,0,"GZ");
		 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
		 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);	
		 
		 //��ʾ����2��3��4����ʾ����//
		 //���֡�ȫ���֡���������ʾ���A\B\C\D��Ŀ���ٶȺ͵�ǰʵ���ٶ�
			OLED_ShowString(0,10,"A");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			
			OLED_ShowString(0,30,"C");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
				
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			
			
			OLED_ShowString(0,40,"D");
			if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
														OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
			else                 	OLED_ShowString(15,40,"+"),
														OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
			if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
														OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,40,"+"),
														OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			
			
			if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			else                       OLED_ShowString(0,50,"ROS");
			
	 		if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
			else                      OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
			                          OLED_ShowString(88,50,".");
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			                          OLED_ShowString(110,50,"V");
		 if(Voltage_Show%100<10) 		OLED_ShowNumber(92,50,0,2,12);
			
		OLED_Refresh_Gram();
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //�Ե�ص�ѹ����ɰٷֱ���ʽ
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //���ڽ����ӡAPP���ݺ���ʾ����
	 flag_show=!flag_show;
	
		if(flag_show==0) {
		 //���Ͳ�����APP��APP����ҳ��ʾ
		 printf(" %d %d ",Voltage_Show, Guangdian_Flag);
	 }

}


