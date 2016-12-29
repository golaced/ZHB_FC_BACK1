
#include "include.h"
#include "usart_fc.h"
#include "ultrasonic.h"
#include "hml5833l.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "rc.h"
#include "att.h"
#include "height_ctrl.h"
#include "alt_kf.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "circle.h"
#include "error.h"
#include "flow.h"


void Usart2_Init(u32 br_num)//--GOL-link
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART2, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
RESULT color;
float dt_flow_rx;
u16 data_rate_gol_link;
PID_STA HPID,SPID,FIX_PID,NAV_PID;
struct _PID_SET pid;
RC_GETDATA Rc_Get;
struct _plane plane;
struct _slam slam;
struct _IMU_NAV imu_nav;
struct _MODE mode;
u8 LOCK, KEY[8],KEY_SEL[4];
struct _FLOW_DEBUG flow_debug;
u8 NAV_BOARD_CONNECT=0;
int RC_OUT[4];
int ERR_GPS[2];
u16 HEIGHT,EX_HEIGHT,ALT_POS_SONAR_HEAD_LASER_SCANER,EX_AVOID,EX_FLOW,NOW_FLOW;
u8 state_fc,state_m100,state_use_m100;
u8 laser_sel;
u8 m100_refresh;
float m100_yaw;
 void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
   if(*(data_buf+2)==0x01)//IMU_FRAME
  {

	  fly_ready=		  *(data_buf+4);	
		if(!mpu6050.Acc_CALIBRATE&&*(data_buf+5)==1)
		mpu6050.Acc_CALIBRATE=1;
		if(!mpu6050.Gyro_CALIBRATE&&*(data_buf+6)==1)
		mpu6050.Gyro_CALIBRATE=1;
		if(!ak8975.Mag_CALIBRATED&&*(data_buf+7)==1)
		ak8975.Mag_CALIBRATED=1;
	}
		else if(*(data_buf+2)==0x14)//IMU_FRAME
  {
	
	

	}
		else if(*(data_buf+2)==0x15)//TEST FRAME
  {
		RC_OUT[0]=LIMIT(((vs16)(*(data_buf+4)<<8)|*(data_buf+5)),1000,2000)-1520;	
		RC_OUT[1]=LIMIT(((vs16)(*(data_buf+6)<<8)|*(data_buf+7)),1000,2000)-1520;	
		RC_OUT[2]=LIMIT(((vs16)(*(data_buf+8)<<8)|*(data_buf+9)),1000,2000)-1520;	
		RC_OUT[3]=LIMIT(((vs16)(*(data_buf+10)<<8)|*(data_buf+11)),1000,2000)-1520;	
    HEIGHT=				  ((vs16)(*(data_buf+12)<<8)|*(data_buf+13));
		EX_HEIGHT=		  ((vs16)(*(data_buf+14)<<8)|*(data_buf+15));
		ALT_POS_SONAR_HEAD_LASER_SCANER= ((vs16)(*(data_buf+16)<<8)|*(data_buf+17));
		EX_AVOID= ((vs16)(*(data_buf+18)<<8)|*(data_buf+19));
		NOW_FLOW= ((vs16)(*(data_buf+20)<<8)|*(data_buf+21));
		EX_FLOW=  ((vs16)(*(data_buf+22)<<8)|*(data_buf+23));
		ERR_GPS[0]= ((vs16)(*(data_buf+24)<<8)|*(data_buf+25));
		ERR_GPS[1]= ((vs16)(*(data_buf+26)<<8)|*(data_buf+27));
		state_fc=				*(data_buf+28);
		state_m100=     *(data_buf+29);
		state_use_m100= *(data_buf+30);
		m100_yaw=(float)((int16_t)(*(data_buf+31)<<8)|*(data_buf+32))/10.;
		laser_sel= *(data_buf+33);
		m100_refresh= *(data_buf+34);
	}
}
 


u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0; 

u8 Rx_Buf[256];	//���ڽ��ջ���
u8 RxBuffer[50];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
u8 com_data ;
static u8 _data_len = 0,_data_cnt = 0;
void USART2_IRQHandler(void)
{ OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART2->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//����жϱ�־

		com_data = USART2->DR;
				if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

   OSIntExit(); 

}

void UsartSend_GOL_LINK(uint8_t ch)
{


	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}

static void Send_Data_GOL_LINK(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK(dataToSend[i]);
}


//------------------------------------------------------GOL_LINK----------------------------------------------------
#include "ekf_ins.h"
void Send_UKF1(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = laser_check[0];//vs16) (flow.rate);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( imu_nav.flow.speed.x*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  (vs16)(imu_nav.flow.speed.y*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[0]*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (flow_ground_temp[1]*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	_temp = (vs16)( FLOW_POS_X*100);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16) (FLOW_POS_Y*100);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	//-------------------
	_temp = (vs16)(ALT_VEL_SONAR*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(ALT_POS_SONAR2*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ALT_VEL_BMP*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
  _temp = (vs16)( ALT_VEL_BMP_EKF*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32) (ALT_POS_BMP_EKF*1000);//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	 _temp = (vs16)( Strength_BEI);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
  _temp=laser_check[0];
	data_to_send[_cnt++]=BYTE0(_temp);
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void Send_CAL(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = (vs16)(mpu6050.Acc_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( mpu6050.Gyro_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Gyro_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Gyro_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ak8975.Mag_Offset.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Offset.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Offset.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp=  (vs16)( ak8975.Mag_Gain.x*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Gain.y*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Gain.z*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_MEMS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x10;//������
	data_to_send[_cnt++]=0;//������

	_temp = (vs16)(mpu6050.Acc.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(mpu6050.Acc.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(mpu6050.Acc.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( mpu6050.Gyro.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( mpu6050.Gyro.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( mpu6050.Gyro.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ak8975.Mag_Val.x);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ak8975.Mag_Val.y);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ak8975.Mag_Val.z);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 = (vs32)(baroAlt);//ultra_distance;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_ATT(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x11;//������
	data_to_send[_cnt++]=0;//������

	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(dj_pit*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)(dj_roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw_mid_down*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)( ref_q_imd_down[0]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = (vs16)( ref_q_imd_down[1]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[2]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)( ref_q_imd_down[3]*1000);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

  _temp = (vs16)( ALT_VEL_BMP_EKF*1000);//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK(data_to_send, _cnt);
}  

void Send_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x12;//������
	data_to_send[_cnt++]=0;//������

	_temp =  en_ble_debug;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  ax;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  ay;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  gx;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  gy;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  gz;//navUkfData.posE[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  hx;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  hy;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Send_Data_GOL_LINK(data_to_send, _cnt);
}

void GOL_LINK_TASK(void)//5ms
{
static u8 cnt[4];

	
//������ֵ
Send_MEMS();//2.5ms
if(cnt[0]++>60)
{cnt[0]=0;
Send_CAL();
}
if(cnt[1]++>0){//5ms
//��̬����
	cnt[1]=0;
Send_ATT();
}

if(cnt[2]++>2)//10ms
{cnt[2]=0;
Send_UKF1();
}

}


void Uart5_Init(u32 br_num)//-----video sonar F
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//����PC12��ΪUART5��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//����PD2��ΪUART5��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//����UART5
	//�жϱ�������
	USART_InitStructure.USART_BaudRate = br_num;       //�����ʿ���ͨ������վ����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
	USART_Init(UART5, &USART_InitStructure);
	


	//ʹ��UART5�����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//ʹ��USART5
	USART_Cmd(UART5, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

void UART5_IRQHandler(void)
{
	u8 com_data;
 OSIntEnter();    
  //�����ж�
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//����жϱ�־

		com_data = UART5->DR;
		
		FLOW_MAVLINK(com_data);//Ultra_Get(com_data);
	}

	//���ͣ�������λ���ж�
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //дDR����жϱ�־
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}
     OSIntExit();    
}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, data_num); ;//USART1, ch); 

}



void Usart1_Init(u32 br_num)//-------GPS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

//	//����PA3  WK  BLE���ƶ�
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure); 
//	
//	GPIO_ResetBits(GPIOA,GPIO_Pin_3);//͸��ģʽ

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART1, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
float dj_pit,dj_roll;
float dlf_odroid=0.5;
 void Data_Receive_Anl1(u8 *data_buf,u8 num)
{
vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
  if(*(data_buf+2)==0x11)//Att
  { //dt_flow_rx= Get_Cycle_T(GET_T_FLOW_UART);	
	  dj_pit=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/10.;
    dj_roll=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/10.;
//		Yaw=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/10.;

	}
		
}
u8 TxBuffer_u1[256];
u8 TxCounter_u1=0;
u8 count_u1=0; 

u8 Rx_Buf1[256];	//���ڽ��ջ���
u8 RxBuffer1[50];
u8 RxState1= 0;
u8 RxBufferNum1 = 0;
u8 RxBufferCnt1 = 0;
u8 RxLen1 = 0;
static u8 _data_len1 = 0,_data_cnt1 = 0;

//GPS
#include "gps.h"
u16 USART3_RX_STA=0;
//���ڷ��ͻ����� 	
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
u8 USART1_TX_BUF[USART3_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 		
u8  buf_GNVTG[65];//GPS��Ϣ
u8 RxBuffertest[50];
u8 cnt_test;
u16 Strength_BEI,ultra_distance_beix;
void USART1_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	static u8 cnt,state,buf[7];
  u8 check,i;
  float temp1;
	if(USART1->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART1->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//����жϱ�־

		com_data = USART1->DR;
	if(!state_use_m100){
		// Laser_Get( com_data);
		 //Ultra_Get( com_data);
		static u8 cnt1;
		RxBuffer1[cnt1++]=com_data;
		if(cnt1>25)cnt1=0;
				if(RxState1==0&&com_data==0xAA)
		{
			RxState1=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState1==1&&com_data==0xAF)
		{
			RxState1=2;
			RxBuffer1[1]=com_data;
		}
		else if(RxState1==2&&com_data>0&&com_data<0XF1)
		{
			RxState1=3;
			RxBuffer1[2]=com_data;
		}
		else if(RxState1==3&&com_data<50)
		{
			RxState1 = 4;
			RxBuffer1[3]=com_data;
			_data_len1 = com_data;
			_data_cnt1 = 0;
		}
		else if(RxState1==4&&_data_len1>0)
		{
			_data_len1--;
			RxBuffer1[4+_data_cnt1++]=com_data;
			if(_data_len1==0)
				RxState1 = 5;
		}
		else if(RxState1==5)
		{
			RxState1 = 0;
			RxBuffer1[4+_data_cnt1]=com_data;
			Data_Receive_Anl1(RxBuffer1,_data_cnt1+5);
		}
		else
			RxState1 = 0;
	}else{
		
		

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
			Strength_BEI=buf[3]<<8|buf[2];

			if(Strength_BEI>10)
			{ temp1=limit_mine(Roll,45);
			ultra_distance_beix=LIMIT((buf[1]<<8|buf[0])*cos(temp1*0.017)*10,0,10000);laser_check[0]=1;laser_cnt[0]=0;}
			}

			}
			}
		}
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART1,USART_IT_TXE ) )
	{
				
		USART1->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}

		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}
 OSIntExit();        
}

void Usart4_Init(u32 br_num)//-------SD_board
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
//	//�����ж����ȼ�
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(UART4, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(UART4, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

void UsartSend_SD(uint8_t ch)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); ;//USART1, ch); 
}

static void Send_Data_SD(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_SD(dataToSend[i]);
}

void UART2_Put_Char(unsigned char DataToSend)
{

while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, DataToSend); 

}

void UART2_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+8);
	UART2_Put_Char(0xA2);

	if(ax<0)ax=32768-ax;
	ctemp=ax>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ax;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(ay<0)ay=32768-ay;
	ctemp=ay>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=ay;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(az<0)az=32768-az;
	ctemp=az>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=az;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gx<0)gx=32768-gx;
	ctemp=gx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(gy<0)gy=32768-gy;
	ctemp=gy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(gz<0)gz=32768-gz;
	ctemp=gz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=gz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hx<0)hx=32768-hx;
	ctemp=hx>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hx;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hy<0)hy=32768-hy;
	ctemp=hy>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hy;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(hz<0)hz=32768-hz;
	ctemp=hz>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=hz;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}


void UART2_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	UART2_Put_Char(0xa5);
	UART2_Put_Char(0x5a);
	UART2_Put_Char(14+4);
	UART2_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;							
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
								 
	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;				
	ctemp=alt;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART2_Put_Char(ctemp);	   
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=IMUpersec>>8;
	UART2_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=IMUpersec;
	UART2_Put_Char(ctemp);
	temp+=ctemp;

	UART2_Put_Char(temp%256);
	UART2_Put_Char(0xaa);
}



//------------------------------------------------------GOL_LINK_SD----------------------------------------------------
void Send_IMU_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x05;//������
	data_to_send[_cnt++]=0;//������
	_temp = Roll*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	

data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_ATT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	_temp = except_A.y*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.x*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = except_A.z*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//������
	data_to_send[_cnt++]=0;//������
	_temp = target_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = target_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = now_position[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = tar_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = actual_speed[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[ROLr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = nav[PITr]*10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_ALT_PID_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//������
	data_to_send[_cnt++]=0;//������
	_temp = exp_height;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = 0;//baroAlt	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_MODE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x04;//������
	data_to_send[_cnt++]=0;//������
	_temp = mode.en_sd_save;
	data_to_send[_cnt++]=BYTE0(_temp);

	
	
  data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

void Send_FLOW_USE_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x06;//������
	data_to_send[_cnt++]=0;//������
	_temp = imu_nav.flow.speed.east ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.west;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  imu_nav.flow.speed.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.speed.y ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = imu_nav.flow.position.east	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = imu_nav.flow.position.west	;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_SLAM_SD(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x07;//������
	data_to_send[_cnt++]=0;//������
	_temp = slam.dis[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = slam.dis[4];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	

	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}


void Send_GPS_SD(void)
{u8 i;	u8 sum = 0;u8 data_to_send[50]={0};
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x08;//������
	data_to_send[_cnt++]=0;//������
	_temp32 =  imu_nav.gps.J;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.W;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp =imu_nav.gps.gps_mode;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =imu_nav.gps.star_num;
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp32 =  imu_nav.gps.X_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_O;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.X_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	_temp32 =  imu_nav.gps.Y_UKF;
	data_to_send[_cnt++]=BYTE3(_temp32);
	data_to_send[_cnt++]=BYTE2(_temp32);
	data_to_send[_cnt++]=BYTE1(_temp32);
	data_to_send[_cnt++]=BYTE0(_temp32);
	
	

	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_SD(data_to_send, _cnt);
}

//void SD_LINK_TASK(void)
//{
//static u8 cnt[4];
//static u8 flag;
//if(cnt[0]++>1)
//{cnt[0]=0;
//Send_ATT_PID_SD();
//}
//if(cnt[1]++>0)
//{cnt[1]=0;
////	if(flag)
////	{flag=0;
//	Send_IMU_SD();
////}
////	else{flag=1;
////	Send_MODE_SD();
////	}
//}
//if(cnt[2]++>2)
//{cnt[2]=0;
//	//Send_FLOW_PID_SD();
//	Send_FLOW_USE_SD();
//}
//if(cnt[3]++>2)
//{cnt[3]=0;
//	//Send_ALT_PID_SD();
//	Send_GPS_SD();//Send_SLAM_SD();
//}

//}

/*
#define SEND_IMU 0
#define SEND_FLOW 1
#define SEND_GPS 2
#define SEND_ALT 3
*/
void SD_LINK_TASK2(u8 sel)
{
static u8 cnt[4];
static u8 flag;
	
	switch(sel)
	{
		case SEND_IMU:
				Send_IMU_SD();
		break;
		case SEND_FLOW:
				Send_FLOW_USE_SD();
		break;
		case SEND_GPS:
				Send_GPS_SD();
		break;
		case SEND_ALT:
				Send_ALT_PID_SD();
		break;
	}

 Send_MODE_SD();
}

void Usart3_Init(u32 br_num)//-------SONAR
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //����USART2ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//�����ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	
	//����PD5��ΪUSART2��Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	//����PD6��ΪUSART2��Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	
   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = br_num;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	

	//ʹ��USART2�����ж�
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//ʹ��USART2
	USART_Cmd(USART3, ENABLE); 
//	//ʹ�ܷ��ͣ�������λ���ж�
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}
#include "sonar_avoid.h"
float rate_gps_board;
 void Data_Receive_Anl3(u8 *data_buf,u8 num)
{
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//�ж�sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//�ж�֡ͷ
	if(*(data_buf+2)==0x01)//SONAR
  {
	//rate_gps_board=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));///10.;
	sonar_avoid[0]=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	sonar_avoid[1]=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	sonar_avoid[2]=((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	sonar_avoid[3]=((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	sonar_avoid[4]=((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	sonar_avoid[5]=((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));
	sonar_avoid[6]=((int16_t)(*(data_buf+16)<<8)|*(data_buf+17));
	sonar_avoid[7]=((int16_t)(*(data_buf+18)<<8)|*(data_buf+19));
	sonar_avoid_c[0]=((int16_t)(*(data_buf+20)<<8)|*(data_buf+21));
	sonar_avoid_c[1]=((int16_t)(*(data_buf+22)<<8)|*(data_buf+23));	
	sys.avoid=1;
	}	
	else if(*(data_buf+2)==0x02)//SLAM_frame
  {
  imu_nav.gps.Y_UKF=((int16_t)(*(data_buf+4)<<8)|*(data_buf+5));
	imu_nav.gps.X_UKF=((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
	imu_nav.gps.Y_O=  ((int16_t)(*(data_buf+8)<<8)|*(data_buf+9));
	imu_nav.gps.X_O=  ((int16_t)(*(data_buf+10)<<8)|*(data_buf+11));
	imu_nav.gps.J=  ((int16_t)(*(data_buf+12)<<8)|*(data_buf+13));
	imu_nav.gps.W=  ((int16_t)(*(data_buf+14)<<8)|*(data_buf+15));	
	sys.gps=1;	
	}			
}
 



u8 Rx_Buf3[256];	//���ڽ��ջ���
u8 RxBuffer3[50];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)
{  OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE�ж�
	{
		com_data = USART3->DR;
	}

  //�����ж�
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//����жϱ�־

		com_data = USART3->DR;
		
		// Laser_Get( com_data);
		 Ultra_Get( com_data);
		static u8 cnt1;
		RxBuffer3[cnt1++]=com_data;
		if(cnt1>25)cnt1=0;
				if(RxState3==0&&com_data==0xAA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xAF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<50)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_Receive_Anl3(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//���ͣ�������λ���ж�
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer[TxCounter++]; //дDR����жϱ�־          
		if(TxCounter == count)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�������жϣ��ж�
		}

		USART_ClearITPendingBit(USART3,USART_IT_TXE);
	}
 OSIntExit();        
}

void UsartSend_GPS(uint8_t ch)
{

while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); ;//USART1, ch); 
}

static void Send_Data_GPS(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GPS(dataToSend[i]);
}

#define MAX_Yun_Angle 60
float Angle_Yun[2]={0,0};

void Send_IMU_TO_GPS(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	_temp =  mode.en_sonar_avoid;//ultra_distance;
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	Angle_Yun[0]= -(float)(RX_CH[6]-1000)/1000*MAX_Yun_Angle;//AUX3
	
	_temp = (vs16)(Angle_Yun[0]*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Angle_Yun[1]*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GPS(data_to_send, _cnt);
}

void CPU_LINK_TASK(void)
{
static u8 cnt[4];
static u8 flag;
if(cnt[0]++>0)
{cnt[0]=0;
 Send_IMU_TO_GPS();
}
}



//-------------------------NAV_BOARD_LINK

//------------------------------------------------------GOL_LINK----------------------------------------------------

void UsartSend_GOL_LINK_NAV(uint8_t ch)
{


	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

static void Send_Data_GOL_LINK_NAV(u8 *dataToSend , u8 length)
{
u16 i;
  for(i=0;i<length;i++)
     UsartSend_GOL_LINK_NAV(dataToSend[i]);
}

void Send_IMU_NAV(void)
{u8 i;	u8 sum = 0;
	u8 data_to_send[50];
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//������
	data_to_send[_cnt++]=0;//������
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);


	
	_temp = q_nav[0]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data_GOL_LINK_NAV(data_to_send, _cnt);
}


u8 SendBuff1[SEND_BUF_SIZE1];	//�������ݻ�����
void data_per_uart1(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
u16 i=0; 	
unsigned int temp=0xaF+9;

char ctemp;	
	
SendBuff1[i++]=0xa5;
SendBuff1[i++]=0x5a;
SendBuff1[i++]=14+8;
SendBuff1[i++]=0xA2;

if(ax<0)ax=32768-ax;
ctemp=ax>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ax;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(ay<0)ay=32768-ay;
ctemp=ay>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=ay;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(az<0)az=32768-az;
ctemp=az>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=az;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gx<0)gx=32768-gx;
ctemp=gx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(gy<0)gy=32768-gy;
ctemp=gy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gy;
SendBuff1[i++]=ctemp;
temp+=ctemp;
//-------------------------
if(gz<0)gz=32768-gz;
ctemp=gz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=gz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hx<0)hx=32768-hx;
ctemp=hx>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hx;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hy<0)hy=32768-hy;
ctemp=hy>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hy;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(hz<0)hz=32768-hz;
ctemp=hz>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=hz;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=temp%256;
SendBuff1[i++]=(0xaa);
//
 temp=0xaF+2+2;
SendBuff1[i++]=(0xa5);
SendBuff1[i++]=(0x5a);
SendBuff1[i++]=(14+4);
SendBuff1[i++]=(0xA1);


if(yaw<0)yaw=32768-yaw;
ctemp=yaw>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=yaw;							
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(pitch<0)pitch=32768-pitch;
ctemp=pitch>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=pitch;
SendBuff1[i++]=ctemp;
temp+=ctemp;
							 
if(roll<0)roll=32768-roll;
ctemp=roll>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=roll;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(alt<0)alt=32768-alt;
ctemp=alt>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;				
ctemp=alt;
SendBuff1[i++]=ctemp;
temp+=ctemp;

if(tempr<0)tempr=32768-tempr;
ctemp=tempr>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=tempr;
SendBuff1[i++]=ctemp;	   
temp+=ctemp;

if(press<0)press=32768-press;
ctemp=press>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=press;
SendBuff1[i++]=ctemp;
temp+=ctemp;

ctemp=IMUpersec>>8;
SendBuff1[i++]=ctemp;
temp+=ctemp;
ctemp=IMUpersec;
SendBuff1[i++]=ctemp;
temp+=ctemp;

SendBuff1[i++]=(temp%256);
SendBuff1[i++]=(0xaa);
}


u8 SendBuff2[SEND_BUF_SIZE2];	//�������ݻ�����
void data_per_uart2(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x01;//������
	SendBuff2[_cnt++]=0;//������
	
	_temp = ultra_distance;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);

	
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 en_ble_debug=0;
void GOL_LINK_BLE_DEBUG(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz)
{
 u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp32;
	SendBuff2[_cnt++]=0xAA;
	SendBuff2[_cnt++]=0xAF;
	SendBuff2[_cnt++]=0x12;//������
	SendBuff2[_cnt++]=0;//������

	_temp =  en_ble_debug;
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ax;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  ay;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  az;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gx;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gy;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  gz;//navUkfData.posE[0]*1000;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hx;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hy;
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	_temp =  hz;//navUkfData.posN[0]*1000;//acc_v[1]*1000;//
	SendBuff2[_cnt++]=BYTE1(_temp);
	SendBuff2[_cnt++]=BYTE0(_temp);
	
	
	SendBuff2[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff2[i];
	SendBuff2[_cnt++] = sum;
}

u8 SendBuff3[SEND_BUF_SIZE3];	//�������ݻ�����
void data_per_uart3(void)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;
  SendBuff3[_cnt++]=0xAA;
	SendBuff3[_cnt++]=0xAF;
	SendBuff3[_cnt++]=0x01;//������
	SendBuff3[_cnt++]=0;//������
	
	_temp = (vs16)(Pitch*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Roll*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = (vs16)(Yaw*10);//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	_temp = mpu6050.Acc.x;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = mpu6050.Acc.y;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp = mpu6050.Acc.z;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);

	_temp = q_nav[0]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
 	_temp = q_nav[1]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[2]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	_temp =  q_nav[3]*1000;//ultra_distance;
	SendBuff3[_cnt++]=BYTE1(_temp);
	SendBuff3[_cnt++]=BYTE0(_temp);
	
	SendBuff3[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff3[i];
	SendBuff3[_cnt++] = sum;
}


u8 SendBuff4[SEND_BUF_SIZE4];	//�������ݻ�����
void data_per_uart4(u8 sel)
{
	u8 i;	u8 sum = 0;
	u8 _cnt=0;
	vs16 _temp;


switch(sel){
	case SEND_ALT:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x03;//������
	SendBuff4[_cnt++]=0;//������
	_temp = exp_height;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_dis_lpf;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ultra_ctrl_out;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = wz_speed;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
 	_temp = thr_test	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_BMP*100	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = ALT_POS_SONAR*100	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
  _temp = mode.en_sd_save;
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	SendBuff4[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	case SEND_IMU:
	SendBuff4[_cnt++]=0xAA;
	SendBuff4[_cnt++]=0xAF;
	SendBuff4[_cnt++]=0x05;//������
	SendBuff4[_cnt++]=0;//������
	_temp = Roll*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Pitch*10;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp = Yaw*10	;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Gyro_I16.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.x;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.y;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=mpu6050.Acc_I16.z;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[0]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[1]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[2]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	_temp=ref_q[3]*1000;
	SendBuff4[_cnt++]=BYTE1(_temp);
	SendBuff4[_cnt++]=BYTE0(_temp);
	
	SendBuff4[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += SendBuff4[i];
	SendBuff4[_cnt++] = sum;
	break;
	default:break;
}
}



//----------------FLOW
#include "led_fc.h"
#include "flow.h"
u8 mav_flow_sel;
FLOW flow;
FLOW_RAD flow_rad;

u8 FLOW_STATE[4];
u8 flow_buf[27];
u8 flow_buf_rad[45];
   float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}

void FLOW_MAVLINK(unsigned char data)
{
/*
��ɫ������ʼ��־λ��stx������v1.0�汾���ԡ�FE����Ϊ��ʼ��־�������־λ��mavlink��Ϣ֡���ն˽�����Ϣ����ʱ���ô���

�ڶ������Ӵ�����ǻ�ɫ���֣�payload��������Ч�غɣ�Ҫ�õ���������Ч�غ����棩���ֽڳ��ȣ�len������Χ��0��255֮�䡣��mavlink��Ϣ֡���ն˿���������ʵ���յ�����Ч�غɵĳ��ȱȽϣ�����֤��Ч�غɵĳ����Ƿ���ȷ��

���������Ӵ�����Ǳ�����Ϣ֡����ţ�seq����ÿ�η���һ����Ϣ������ֽڵ����ݻ��1���ӵ�255����0���¿�ʼ������������mavlink��Ϣ֡���ն˼�����Ϣ��ʧ�����õģ��൱�����ź�ǿ�ȡ�

���ĸ����Ӵ����˷��ͱ�����Ϣ֡���豸��ϵͳ��ţ�sys����ʹ��PIXHAWKˢPX4�̼�ʱĬ�ϵ�ϵͳ���Ϊ1������mavlink��Ϣ֡���ն�ʶ�����ĸ��豸��������Ϣ��

��������Ӵ����˷��ͱ�����Ϣ֡���豸�ĵ�Ԫ��ţ�comp����ʹ��PIXHAWKˢPX4�̼�ʱĬ�ϵĵ�Ԫ���Ϊ50������mavlink��Ϣ֡���ն�ʶ�����豸���ĸ���Ԫ��������Ϣ����ʱûʲô�ã� ��

���������Ӵ�������Ч�غ�����Ϣ���ı�ţ�msg����ע����������ǲ�ͬ�ģ�����ֽں���Ҫ��mavlink��Ϣ֡���ն�Ҫ������������ȷ����Ч�غ��ﵽ�׷���ʲô��Ϣ�������ݱ��ѡ���Ӧ�ķ�ʽ��������Ч�غ������Ϣ����
      26*/		 
// FE 1A| A2 X X| 64
	
static u8 s_flow=0,data_cnt=0;
float sonar_new;
static float  temp,sonar_lp;
u8 cnt_offset=0;	
u8 get_one_fame=0;
char floattobyte[4];
		switch(s_flow)
	 {
    case 0: if(data==0xFE)
			s_flow=1;
			break;
		case 1: if(data==0x1A||data==0x2C)
				{ s_flow=2;}
			else
			s_flow=0;
			break;
	  case 2:
			if(data_cnt<4)
			{s_flow=2; FLOW_STATE[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=3;flow_buf[data_cnt++]=data;}
		 break;
		case 3:
		 if(FLOW_STATE[3]==100){
			if(data_cnt<26)
			{s_flow=3; flow_buf[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else if(FLOW_STATE[3]==106){
			if(data_cnt<44)
			{s_flow=3; flow_buf_rad[data_cnt++]=data;}
		  else
			{data_cnt=0;s_flow=4;}
		}
		else
			{data_cnt=0;s_flow=0;}
			 break;
		case 4:get_one_fame=1;s_flow=0;data_cnt=0;break;
		default:s_flow=0;data_cnt=0;break;
	 }//--end of s_uart
		

	 if(get_one_fame)
	 { mav_flow_sel=2;
		 flow.flow_cnt++;
		 if(FLOW_STATE[3]==100){
		flow.time_sec=(flow_buf[7]<<64)|(flow_buf[6]<<56)|(flow_buf[5]<<48)|(flow_buf[4]<<40)
		 |(flow_buf[3]<<32)|(flow_buf[2]<<16)|(flow_buf[1]<<8)|(flow_buf[0]);
  	 floattobyte[0]=flow_buf[8];
		 floattobyte[1]=flow_buf[9];
		 floattobyte[2]=flow_buf[10];
		 floattobyte[3]=flow_buf[11];
		flow.flow_comp_x.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[12];
		 floattobyte[1]=flow_buf[13];
		 floattobyte[2]=flow_buf[14];
		 floattobyte[3]=flow_buf[15];
		flow.flow_comp_y.originf =ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf[16];
		 floattobyte[1]=flow_buf[17];
		 floattobyte[2]=flow_buf[18];
		 floattobyte[3]=flow_buf[19];
//	   if(!GOL_LINK_CONNECT||height_ctrl_mode!=2||x_pred==0)
//			 sonar_new=1;
//		 else
//			 sonar_new=x_pred;//ByteToFloat(floattobyte);//ground_distance	float	Ground distance in m. Positive value: distance known. Negative value: Unknown distance     
	  
		 flow.hight.originf= 0.05f * sonar_new + 0.95f * sonar_lp;
		 sonar_lp = sonar_new;
		 flow.flow_x.origin=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
	   flow.flow_y.origin=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
		 flow.id=flow_buf[24];
	   flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
	//	flow_fix.flow_comp_x.average 		 = IIR_I_Filter(flow_fix.flow_comp_x.originf , InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_y.average 		 = IIR_I_Filter(flow.flow_y.origin , InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_x.average = IIR_I_Filter(flow.flow_comp_x.originf , InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	//	flow.flow_comp_y.average = IIR_I_Filter(flow.flow_comp_y.originf , InPut_IIR[3], OutPut_IIR[3], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
    flow.new_data_flag	=1;LED1=!LED1;
		 }
	 else if(FLOW_STATE[3]==106)
	 {
	 	flow_rad.time_usec=(flow_buf_rad[7]<<64)|(flow_buf_rad[6]<<56)|(flow_buf_rad[5]<<48)|(flow_buf_rad[4]<<40)
		 |(flow_buf_rad[3]<<32)|(flow_buf_rad[2]<<16)|(flow_buf_rad[1]<<8)|(flow_buf_rad[0]);
  	 flow_rad.integration_time_us=(flow_buf_rad[11]<<32)|(flow_buf_rad[10]<<16)|(flow_buf_rad[9]<<8)|(flow_buf_rad[8]);
		 floattobyte[0]=flow_buf_rad[12];
		 floattobyte[1]=flow_buf_rad[13];
		 floattobyte[2]=flow_buf_rad[14];
		 floattobyte[3]=flow_buf_rad[15];
		 flow_rad.integrated_x=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[16];
		 floattobyte[1]=flow_buf_rad[17];
		 floattobyte[2]=flow_buf_rad[18];
		 floattobyte[3]=flow_buf_rad[19];
		 flow_rad.integrated_y=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[20];
		 floattobyte[1]=flow_buf_rad[21];
		 floattobyte[2]=flow_buf_rad[22];
		 floattobyte[3]=flow_buf_rad[23];
		 flow_rad.integrated_xgyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[24];
		 floattobyte[1]=flow_buf_rad[25];
		 floattobyte[2]=flow_buf_rad[26];
		 floattobyte[3]=flow_buf_rad[27];
		 flow_rad.integrated_ygyro=ByteToFloat(floattobyte);
		 floattobyte[0]=flow_buf_rad[28];
		 floattobyte[1]=flow_buf_rad[29];
		 floattobyte[2]=flow_buf_rad[30];
		 floattobyte[3]=flow_buf_rad[31];
		 flow_rad.integrated_zgyro=ByteToFloat(floattobyte);
		 flow_rad.time_delta_distance_us=(flow_buf_rad[35]<<32)|(flow_buf_rad[34]<<16)|(flow_buf_rad[33]<<8)|(flow_buf_rad[32]);
		 floattobyte[0]=flow_buf_rad[36];
		 floattobyte[1]=flow_buf_rad[37];
		 floattobyte[2]=flow_buf_rad[38];
		 floattobyte[3]=flow_buf_rad[39];
		 flow_rad.distance=ByteToFloat(floattobyte);
		 flow_rad.temperature=(flow_buf_rad[41]<<8)|(flow_buf_rad[40]);
		 flow_rad.sensor_id=(flow_buf_rad[42]);
		 flow_rad.quality=(flow_buf_rad[43]);
		 
    //flow_task();
		 
	 }	
	 }
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

