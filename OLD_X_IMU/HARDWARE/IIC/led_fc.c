

#include "led_fc.h"
#include "include.h"
#include "mpu6050.h"
#include "hml5833l.h"
void LEDRGB_COLOR(u8 color);
void LED_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LED1| ANO_Pin_LED2| ANO_Pin_LED3| ANO_Pin_LED4;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);
	
	LEDRGB_COLOR(BLUE);
	Delay_ms(500);
	LEDRGB_COLOR(RED);
	Delay_ms(500);
	LEDRGB_COLOR(GREEN);
  Delay_ms(500);



	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}



/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/


void LEDRGB(u8 sel,u8 on)
{
switch(sel)
{
case BLUE:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_0);
else
GPIO_SetBits(GPIOE,GPIO_Pin_0);
break;
case RED:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_1);
else
GPIO_SetBits(GPIOE,GPIO_Pin_1);
break;
case  GREEN:
if(!on)
GPIO_ResetBits(GPIOE,GPIO_Pin_2);
else
GPIO_SetBits(GPIOE,GPIO_Pin_2);
break;
}
}

void LEDRGB_COLOR(u8 color)
{
switch (color)
{
case RED:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case BLUE:
LEDRGB(RED,0);
LEDRGB(BLUE,1);
LEDRGB(GREEN,0);
break;
case GREEN:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
case WHITE:
LEDRGB(RED,1);
LEDRGB(BLUE,1);
LEDRGB(GREEN,1);
break;
case BLACK:
LEDRGB(RED,0);
LEDRGB(BLUE,0);
LEDRGB(GREEN,0);
break;
case YELLOW:
LEDRGB(RED,1);
LEDRGB(BLUE,0);
LEDRGB(GREEN,1);
break;
}
}
#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2

#define IDLE 0
#define CAL_MPU 1
#define CAL_M  2
#include "circle.h"
float SET_YAW=0.7;
void LEDRGB_STATE(void)
{
static u8 main_state;
static u8 mpu_state,m_state,idle_state;
static u16 cnt,cnt_idle;
u8 mode_control;
	
	
mode_control=mode.mode_fly;
switch(main_state)
{ 
	
	case IDLE:
	if(mpu6050.Gyro_CALIBRATE)
	{idle_state=0;main_state=CAL_MPU;

	}
	else if(Mag_CALIBRATED)
	{idle_state=0;main_state=CAL_M;

	}
	break;
	case CAL_MPU:
  break;
	case CAL_M:
  break;
}
//   | | | |    | | | |   | | | |   | | | |   | | | |
//    ARM          GPS1     GPS2      GPS3      MODE  
#define RGB_DELAY 3


switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(fabs(To_180_degrees(m100_yaw-SET_YAW))<0.5)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>0.5/0.05)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>0.5/0.05)
	{idle_state=0;cnt_idle=0;}
	break;
}
/*
switch(idle_state)
{//ARM
	case 0:
		if(main_state==IDLE)
			{idle_state=1;cnt_idle=0;}
	break;
	case 1:
	 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY*2)
	{idle_state=2;cnt_idle=0;}
	break;
	case 2:
		LEDRGB_COLOR(BLACK);	
	if(cnt_idle++>RGB_DELAY)
	{idle_state=3;cnt_idle=0;}
	break;
	case 3:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=4;cnt_idle=0;}
	break;
//GPS1	
	case 4:
		 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=5;cnt_idle=0;}
	break;
	case 5:
		 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=6;cnt_idle=0;}
	break;
	case 6:
		 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=7;cnt_idle=0;}
	break;
	case 7:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=8;cnt_idle=0;}
	break;
		
//GPS2
	case 8:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=9;cnt_idle=0;}
	break;
	case 9:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=10;cnt_idle=0;}
	break;
	case 10:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=11;cnt_idle=0;}
	break;
	case 11:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=12;cnt_idle=0;}
	break;
//GPS3	
	case 12:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=13;cnt_idle=0;}
	break;
	case 13:
		 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=14;cnt_idle=0;}
	break;
	case 14:
		 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=15;cnt_idle=0;}
	break;
	case 15:
			 if(circle.connect==1)
				LEDRGB_COLOR(WHITE); 
		 else
				LEDRGB_COLOR(RED);
	if(cnt_idle++>RGB_DELAY)
	{idle_state=16;cnt_idle=0;}
	break;
//MODE
	case 16:
		switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(YELLOW);break;//zit
			 case 1:  LEDRGB_COLOR(GREEN);break;//gps
		 }
		
	if(cnt_idle++>RGB_DELAY)
	{idle_state=17;cnt_idle=0;}
	break;
	case 17:
			switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(YELLOW);break;//zit
			 case 1:  LEDRGB_COLOR(GREEN);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=18;cnt_idle=0;}
	break;
	case 18:
			switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(BLACK);break;//zit
			 case 1:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=19;cnt_idle=0;}
	break;
	case 19:
				switch(mode_control)
	   {
			 case 0:	LEDRGB_COLOR(BLACK);break;//zit
			 case 1:  LEDRGB_COLOR(BLACK);break;//gps
		 }
	if(cnt_idle++>RGB_DELAY)
	{idle_state=20;cnt_idle=0;}
	break;
//-END
	case 20:
			LEDRGB_COLOR(BLACK);
	if(cnt_idle++>20)
	{idle_state=0;cnt_idle=0;}
	break;
}

switch(mpu_state){
	case 0:if(main_state==CAL_MPU)
	  {mpu_state=1;LEDRGB_COLOR(YELLOW);}
	       break;
	case 1:
		if(!mpu6050.Gyro_CALIBRATE)
		{	mpu_state=2;LEDRGB_COLOR(BLACK);}
		    break;
	case 2:
		 if(cnt++>10)
		 {mpu_state=0;cnt=0;main_state=IDLE;}
		 break;
	default:mpu_state=0;break;
	 }	 

switch(m_state){
	case 0:if(main_state==CAL_M)
	  {m_state=1;LEDRGB_COLOR(BLUE);}
	       break;
	case 1:
			if(!Mag_CALIBRATED)
		{	m_state=2;LEDRGB_COLOR(GREEN);}
		  break;
	case 2:
		 if(cnt++>10)
		 {m_state=0;cnt=0;main_state=IDLE;}
		 break;
	default:m_state=0;break;
	 }	 	 */
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/



/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

