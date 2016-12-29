#include "include.h"
#include "ultrasonic.h"
#include "usart.h"
#include "filter.h"
#include "IMU.h"
#include "rc.h"
#include "ms5611.h"
#include "myiic_sonar.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "error.h"
float T_sonar;

void SONAR_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	//����LEDʹ�õ��ùܽ�
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	/*����GPIOB������ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOFʱ��
	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7	;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����

	/*���ÿ⺯������ʼ��GPIOB*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);		 

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	/*���ÿ⺯������ʼ��GPIOB*/
	GPIO_Init(GPIOE, &GPIO_InitStructure);		 
	GPIO_ResetBits(GPIOE,GPIO_Pin_6);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource7);//PE2 ���ӵ��ж���2
	/* ����EXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش��� 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
	EXTI_Init(&EXTI_InitStructure);//����

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�4
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����
}
void GPIO_SET_SONAR(u8 num)
{

GPIO_SetBits(GPIOE,GPIO_Pin_6);	
}

void GPIO_RESET_SONAR(u8 num)
{
GPIO_ResetBits(GPIOE,GPIO_Pin_6);
}
u8 GPIO_READ_SONAR(u8 num)
{u8 temp=0;

temp=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7);
return temp;
}

void Ultrasonic_Init()
{ 
	#if defined(SONAR_USE_UART)   
	//Uart5_Init(9600);			//����5��ʼ������������Ϊ������
	#endif
	#if defined(SONAR_USE_TIG)    
	SONAR_GPIO_Config();	ultra_ok = 1;
	#endif
	#if defined(SONAR_USE_SCL)    
	IIC_Init_Sonar();
	//KS10X_Change_Addr_Init(SlaveAddress3,SlaveAddress2);  
	//ultra_ok=AT24CXX_Check();	
	#endif
}

s8 ultra_start_f;
u8 ultra_time;
u8 ultra_ok = 0;

void Laser_Duty()
{u8 temp[3];
	static u8 cnt;
//3a 00 22 01 0d 0a
//if(cnt++>10){
//cnt=0;
//Uart5_Send(temp ,0x3a);
//Uart5_Send(temp ,0x00);
//Uart5_Send(temp ,0x21);
//Uart5_Send(temp ,0x28);
//Uart5_Send(temp ,0x0d);
//Uart5_Send(temp ,0x0a);
//}
//else{	
//#if defined(USE_LASER_BEIX)  	
// Uart5_Send(temp ,0x42);
//	 Uart5_Send(temp ,0x57);
//	 Uart5_Send(temp ,0x02);
//	 Uart5_Send(temp ,0x00);
//	 Uart5_Send(temp ,0x00);
//	 Uart5_Send(temp ,0x00);
//	 Uart5_Send(temp ,0x00);
//   Uart5_Send(temp ,0xE6);	
//#elsif defined(USE_LASER_MINE)  
if(fly_ready)
   Uart5_Send(temp ,'V');
else
	 Uart5_Send(temp ,'C');
//#else	
//Uart5_Send(temp ,0x3a);
//Uart5_Send(temp ,0x00);
//Uart5_Send(temp ,0x22);
//Uart5_Send(temp ,0x01);
//Uart5_Send(temp ,0x0d);
//Uart5_Send(temp ,0x0a);
//#endif	
//}
}
void Ultra_Duty()
{
	u8 temp[3];

	ultra_time++;
	ultra_time = ultra_time%2;
	
#if defined (SONAR_USE_UART)
	#if defined(USE_KS103)
		Uart5_Send(temp ,0xe8);Delay_us(20);
	  Uart5_Send(temp ,0x02);Delay_us(20);
			 #if   defined(SONAR_SAMPLE1)
				Uart5_Send(temp ,0xb0);
	     #elif defined(SONAR_SAMPLE2)
	      Uart5_Send(temp ,0xbc);
	     #elif defined(SONAR_SAMPLE3)
	      Uart5_Send(temp ,0xb8);
			 #endif
	#elif defined(USE_US100)
		temp[0] = 0x55;
		Uart5_Send(temp ,0x55);
	#endif
#else
	GPIO_SET_SONAR(0);
	Delay_us(20);
	GPIO_RESET_SONAR(0);
#endif	
			
	ultra_start_f = 1;
}

void Ultra_Duty_SCL(void)
{ float temp1,temp,temp2;
	static u16 ultra_distance_old;
  u16 range;
//--
	//KS103_WriteOneByte(0XE8,0X02,0XB0);//5m
	//KS103_WriteOneByte(0XE8,0X02,0XB8);//11m
//---

//	range = KS103_ReadOneByte(0xe8, 0x02);
//	range <<= 8;
//	range += KS103_ReadOneByte(0xe8, 0x03);
	range=Read_KS10X_Data(0XE8);
	temp1=limit_mine(Roll,45);
	temp2=limit_mine(Pitch,45);
	temp=(float)range*cos(temp1*0.017)*cos(temp2*0.017);
	temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
	ultra_distance=Moving_Median(1,5,temp);
	T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE);
	ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.0001,1);

	ultra_distance_old = ultra_distance;
	ultra_start_f = 1;
	delay_ms(20);
}
// float t1r=1;
/* kalman filter states */
double x_pred = 0.0f; // m   0
double v_pred = 0.0f; //       1
double x_post = 0.0f; // m    2
double v_post = 0.0f; // m/s  3
float sonar_raw = 0.0f;  // m
float scale_kal_sonar_v=0.2;
float sonar_filter(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=1;//0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred = x_post + dt_sonar * v_pred;
	v_pred = v_post;
   v_pred=limit_mine	(v_pred,MAX_SPEED);
	 if(fabs(v_pred) < 0.01)           \
    v_pred = 0;   
	 
	 v_pred=reg*(1-LPF_1)+v_pred*(LPF_1);
	 reg=v_pred;
	 x_new = hight;
	sonar_raw = x_new;
	x_post = x_pred +  0.91* (x_new - x_pred);//0.8461f
	v_post = v_pred +  6.2034f* (x_new - x_pred)*scale_kal_sonar_v;
  v_post=limit_mine(v_post,MAX_SPEED);
	  if(fabs(v_post) < 0.01)           \
    v_post = 0;   
	return x_pred;//m/s
}

// float t1r=1;
/* kalman filter states */
double x_pred_bmp = 0.0f; // m   0
double v_pred_bmp = 0.0f; //       1
double x_post_bmp = 0.0f; // m    2
double v_post_bmp = 0.0f; // m/s  3
float sonar_filter_bmp(float hight,float dt_sonar)
{float x_new;
 static float reg;
	float LPF_1=0.75; 
	float MAX_SPEED=0.5;
	/* no data for long time */
	if (dt_sonar > 0.25f) // more than 2 values lost
	{
		v_pred = 0;
	}

	x_pred_bmp = x_post_bmp + dt_sonar * v_pred_bmp;
	v_pred_bmp = v_post_bmp;
   v_pred_bmp=limit_mine	(v_pred_bmp,MAX_SPEED);
	 if(fabs(v_pred_bmp) < 0.01)           \
    v_pred_bmp = 0;   
	 
	 v_pred_bmp=reg*(1-LPF_1)+v_pred_bmp*(LPF_1);
	 reg=v_pred_bmp;
	 x_new = hight;
	x_post_bmp = x_pred_bmp +  0.91* (x_new - x_pred_bmp);//0.8461f
	v_post_bmp = v_pred_bmp +  6.2034f* (x_new - x_pred_bmp)*scale_kal_sonar_v;
  v_post_bmp=limit_mine(v_post_bmp,MAX_SPEED);
	  if(fabs(v_post_bmp) < 0.01)           \
    v_post_bmp = 0;   
	return x_pred_bmp;//m/s
}

u8 state_dj[5]={0,0,0,0,0};
u8 state_dj_rx[5]={0,0,0,0,0};
u8 IO_STATE[5]={0,0,0,0,0};
u8 IO_STATER[5]={0,0,0,0,0};
u32 TEMP_SONAR=340*3000/200;//-------------------------------------������ʱ�����
//�ⲿ�ж�4�������
u32 cnt_sample1,now_dj[4],lastUpdate_dj[4];

void EXTI9_5_IRQHandler(void)
{     OSIntEnter();  
	float temp,temp1,temp2;
		static int ultra_distance_old;
	
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
				{
			IO_STATE[3]=GPIO_READ_SONAR(3);
			 switch(state_dj_rx[3])
			 {
				 case 0:if(IO_STATE[3]==1)
				 { 
					 lastUpdate_dj[3] =  GetSysTime_us();
					 state_dj_rx[3]=1; 
				 }
				 break;
				 case 1:		 
				 now_dj[3] = GetSysTime_us();  //��ȡʱ��
				 if( now_dj[3] <lastUpdate_dj[3]){cnt_sample1 =  ((float)( now_dj[3]  + (0xffff- lastUpdate_dj[3])) );}
				 else	{ cnt_sample1 =  ((float)( now_dj[3]  - lastUpdate_dj[3])); }
					 if(IO_STATE[3]==1)
						 state_dj_rx[3]=0; 
					 else if(IO_STATE[3]==0)
				 {  
				temp1=0;//limit_mine(Roll,45);
				temp2=0;//limit_mine(Pitch,45);
				temp=(float)cnt_sample1*1000/(TEMP_SONAR)*cos(temp1*0.017)*cos(temp2*0.017);
				temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
				ultra_distance=temp;//Moving_Median(1,5,temp);
				ultra_ok = 1;ultra_start_f = 0;
				state_dj_rx[3]=0;
		     sys.sonar=1;
				 T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE);
				//ultra_delta = (ultra_distance - ultra_distance_old)/LIMIT(T_sonar,0.000000001,1);
				//sonar_filter((float) temp/1000,T_sonar);
				//ultra_distance_old = ultra_distance;
				 }
				
				 break;
				 default:
					 state_dj_rx[3]=0;
					 break;
			 }
				
					EXTI_ClearITPendingBit(EXTI_Line7);
	}   
				   OSIntExit();
}

u16 laser_cnt[2]; u8 laser_check[2];
int ultra_distance,ultra_distance_r,Strength;
float ultra_delta;
u8 buf_laser[7];
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp,state;
	float temp1,temp2,temp;
	float dt;
//	static int ultra_distance_old;
//	if( ultra_start_f == 1 )
//	{
//		ultra_tmp = com_data;
//		ultra_start_f = 2;
//	}
//	else if( ultra_start_f == 2 )
//	{
//		temp = (ultra_tmp<<8) + com_data;
//		ultra_start_f = 0; 
//		dt=Get_Cycle_T(GET_T_SONAR_SAMPLE);
//		temp1=limit_mine(Roll,45);
//		temp2=limit_mine(Pitch,45);
//		temp=temp*cos(temp1*0.017)*cos(temp2*0.017);
//		if(temp<3200){
//	  temp=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
//   	//sonar_filter((float) temp/1000.,dt);
//	  //ultra_distance=((x_pred*1000)<(0)?(0):((x_pred*1000)>(2500)?(2500):(x_pred*1000)));
//			//if(height_ctrl_mode==1)
//			//ultra_distance=baroAlt*10;
//			//else
//		
//		 T_sonar=Get_Cycle_T(GET_T_SONAR_SAMPLE); 
//		ultra_distance=temp;//Moving_Median(1,5,temp);
//		}
//		sys.sonar=ultra_ok = 1;
//	}
	 
	
//	 ultra_delta = x_pred*1000 - ultra_distance_old;
//	
//	 ultra_distance_old = x_pred*1000;
static u8 cnt,buf[7];
u8 check,i;
#if defined(USE_LASER_BEIX)  	 

	switch(state)
	{
		case 0:
			if(com_data==0x59)
			{state=1;cnt=0;}
		break;
		case 1:
			if(com_data==0x59)
			{state=2;cnt=0;}
			else 
			 state=0;
		break	;
		case 2:
			 if(cnt<=7)
			  buf[cnt++]=com_data;
			 else
			 {state=0;
				
				 check=0x59+0x59+buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5];
				 if(check==buf[6])
				 {
				 Strength=buf[3]<<8|buf[2];
					 
					 if(Strength>10)
					 { temp1=limit_mine(Roll,45);
						ultra_distance=(buf[1]<<8|buf[0])*cos(temp1*0.017)*10;laser_check[0]=1;laser_cnt[0]=0;}
				 }
				 
			 }
			 break;
	
	}
#else
if(laser_sel)
	ultra_distance=ultra_distance_beix;
else{
switch(state)
	{
		case 0:
			if(com_data==':')
			{state=1;cnt=0;}
		break;
		case 1:
			if(com_data==' ')
			{state=2;cnt=0;}
			else 
			 state=0;
		break	;
		case 2:
			 if(cnt<=6&&com_data!='m')
			  buf_laser[cnt++]=com_data;
			 else 
			 {state=0;
				cnt=0;
	
					  temp1=limit_mine(Roll,45);
						ultra_distance=((buf_laser[0]-48)*1000+(buf_laser[2]-48)*100+(buf_laser[3]-48)*10+(buf_laser[4]-48))*cos(temp1*0.017);laser_check[0]=1;laser_cnt[0]=0;
				  ultra_distance=LIMIT(ultra_distance,0,7000);
				 
			 }
			 break;
	
	}
}	
	
#endif	
}

////D00.63
//u8 laser[30];
//void Laser_Get(u8 com_data)
//{
//	static u8 cnt,state,cnt1;
//	int temp1,temp2,temp;
//  static u8 reg[5];
//	
//	if(cnt1<29)
//		laser[cnt1++]=com_data;
//	else
//		cnt1=0;
//  switch(state)
//	{
//		case 0:
//	   if(com_data=='D')
//		 {state=1;}
//	  break;
//	  case 1:
//	   if(com_data=='0')
//		 {state=2;cnt=0;}
//		 else
//			state=0; 
//	  break;
//		case 2:
//		  reg[cnt++]= com_data;
//		 if(cnt>3)
//		 { cnt=0;state=0;
//			temp=(reg[0]-48)*1000+ (reg[2]-48)*100+(reg[3]-48)*10;
//			temp1=limit_mine(Roll,45);
//			temp2=limit_mine(Pitch,45);
//			temp=temp*cos(temp2*0.017);
//			if(temp<3200)
//			ultra_distance=((temp)<(0)?(0):((temp)>(4500)?(4500):(temp)));
//		 }
//		 break;
//	  }
//	

//}
