#include "include.h" 
#include "iic_soft.h"
#include "hml_sample.h"
#include "ms5611.h"
#include "alt_kf.h"
#include "flash.h"
#include "led_fc.h"
#include "bmp_adc.h"
#include "flash_w25.h"
#include "ucos_ii.h"
#include "os_cpu.h"
#include "os_cfg.h"
#include "ucos_task.h"
#include "stm32f4xx_dma.h"
 /////////////////////////UCOSII������������///////////////////////////////////
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			20 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				64
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);	
//////////////////////////////////////////////////////////////////////////////
    
OS_EVENT * msg_key;			//���������¼���	  
OS_EVENT * q_msg;			//��Ϣ����

OS_FLAG_GRP * flags_key;	//�����ź�����
void * MsgGrp[256];			//��Ϣ���д洢��ַ,���֧��256����Ϣ
uint16_t cpuGetFlashSize(void)
{

   return (*(__IO u16*)(0x1FFF7A22));
   // return (*(__IO u32*)(0x1FFF7A20))>>16;
}

//��ȡChipID
u32 mcuID[3];
void cpuidGetId(void)
{
    mcuID[0] = *(__IO u32*)(0x1FFF7A10);
    mcuID[1] = *(__IO u32*)(0x1FFF7A14);
    mcuID[2] = *(__IO u32*)(0x1FFF7A18);
}


int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_GROUP);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	Initial_Timer_SYS();
	Delay_ms(100);
  I2c_Soft_Init();					//��ʼ��ģ��I2C
	Delay_ms(100);
	MS5611_Init();						//��ѹ�Ƴ�ʼ��
	Delay_ms(100);						//��ʱ
	MPU6050_Init(20);   			//���ٶȼơ������ǳ�ʼ��������20hz��ͨ
	Delay_ms(100);						//��ʱ
	HMC5883L_SetUp();	
	Delay_ms(100);
	MS5611_Init();						//��ѹ�Ƴ�ʼ��
	altUkfInit();
	LED_Init();								//LED���ܳ�ʼ��
	Delay_ms(100);
//------------------------Uart Init-------------------------------------
	Usart1_Init(115200L);			//IMU_DJ   //M100--LASER BWake
	TIM7_Int_Init(100-1,8400-1);		//100ms�ж�
//	#if EN_DMA_UART1 
//	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
//	#endif
	Usart2_Init(576000L);			//IMU_LINK
	#if EN_DMA_UART2
	MYDMA_Config(DMA1_Stream6,DMA_Channel_4,(u32)&USART2->DR,(u32)SendBuff2,SEND_BUF_SIZE2+2,1);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	#endif
	Usart4_Init(115200L);     //UP Link
	//	#if EN_DMA_UART4 
	//	//MYDMA_Config(DMA1_Stream4,DMA_Channel_4,(u32)&UART4->DR,(u32)SendBuff4,SEND_BUF_SIZE4+2,0);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	//	#endif
	#if defined(USE_LASER_BEIX) 
	Usart3_Init(115200L);
	#elif defined(USE_LASER_MINE)
	Usart3_Init(115200L);
	#else
	Usart3_Init(9600L);    		//LASETR
	#endif
	//#if EN_DMA_UART3
	//MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,2);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	// #endif
  Uart5_Init(115200L);			//FLOW
	Delay_ms(100);
//-------------------------Para Init------------------------------------	
	Para_Init();							//������ʼ��
	W25QXX_Init();			//W25QXX��ʼ��
	while(W25QXX_ReadID()!=W25Q32)								//��ⲻ��W25Q128
	{	LEDRGB_COLOR(RED);
		Delay_ms(100);
		LEDRGB_COLOR(BLACK);
		Delay_ms(100);
	}
	READ_PARM();//��ȡ����
	Delay_ms(100);
//-----------------------Mode &  Flag init--------------------	
//--system
	fly_ready=0;
	mode.en_imu_ekf=0;

	RX_CH[PITr]=RX_CH[ROLr]=RX_CH[YAWr]=1500;
	RX_CH[THRr]=1000;
	//--ң����ƫִ
	RX_CH_FIX[ROLr]=4;//-15;
	RX_CH_FIX[PITr]=0;//-1;
	RX_CH_FIX[THRr]=0;//7;
	RX_CH_FIX[YAWr]=1;//11;
	//--
	RX_CH[AUX1r]=500;
	RX_CH[AUX2r]=500;
	RX_CH[AUX3r]=500;
	RX_CH[AUX4r]=500;

	//-----------------DMA Init--------------------------
//#if EN_DMA_UART4 
//	//USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
//	//MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE4+2);     //��ʼһ��DMA���䣡
//#endif
#if EN_DMA_UART2
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA1_Stream6,SEND_BUF_SIZE2+2);     //��ʼһ��DMA���䣡	
#endif
#if EN_DMA_UART3
	//USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	//MYDMA_Enable(DMA1_Stream3,SEND_BUF_SIZE3+2);     //��ʼһ��DMA���䣡	
#endif
#if EN_DMA_UART1 
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     //��ʼһ��DMA���䣡	 
#endif	
  TIM3_Int_Init(25-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms     
	Delay_ms(2000);//�ϵ���ʱ
	//---------------��ʼ��UCOSII--------------------------
	OSInit();  	 				
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	    
}
 

//��ʼ����
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	u8 err;	    	    
	pdata = pdata; 	
	msg_key=OSMboxCreate((void*)0);		//������Ϣ����
	q_msg=OSQCreate(&MsgGrp[0],256);	//������Ϣ����
 	flags_key=OSFlagCreate(0,&err); 	//�����ź�����		  
	  
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������
	//ע�������ʱ��
	tmr1=OSTmrCreate(10,10,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//100msִ��һ��  cpuʹ����
	OSTmrStart(tmr1,&err);//���������ʱ��1				 	
	tmr2=OSTmrCreate(10,5,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr2_callback,0,"tmr2",&err);		//50msִ��һ��  LED&&MODE
	OSTmrStart(tmr2,&err);//���������ʱ��1				 	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
 	//ע���߳� 	
	OSTaskCreate(outer_task,(void *)0,(OS_STK*)&OUTER_TASK_STK[OUTER_STK_SIZE-1],OUTER_TASK_PRIO);
	OSTaskCreate(ekf_task,(void *)0,(OS_STK*)&EKF_TASK_STK[EKF_STK_SIZE-1],EKF_TASK_PRIO);
	OSTaskCreate(flow_task1,(void *)0,(OS_STK*)&FLOW_TASK_STK[FLOW_STK_SIZE-1],FLOW_TASK_PRIO);
	OSTaskCreate(baro_task,(void *)0,(OS_STK*)&BARO_TASK_STK[BARO_STK_SIZE-1],BARO_TASK_PRIO);
	OSTaskCreate(sonar_task,(void *)0,(OS_STK*)&SONAR_TASK_STK[SONAR_STK_SIZE-1],SONAR_TASK_PRIO);	
	OSTaskCreate(uart_task,(void *)0,(OS_STK*)&UART_TASK_STK[UART_STK_SIZE-1],UART_TASK_PRIO);
	OSTaskCreate(error_task,(void *)0,(OS_STK*)&ERROR_TASK_STK[ERROR_STK_SIZE-1],ERROR_TASK_PRIO);
	//--
 	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
   

//�ź�������������
void flags_task(void *pdata)
{	
	u16 flags;	
	u8 err;	    						 
	while(1)
	{
		flags=OSFlagPend(flags_key,0X001F,OS_FLAG_WAIT_SET_ANY,0,&err);//�ȴ��ź���
 		
		OSFlagPost(flags_key,0X001F,OS_FLAG_CLR,&err);//ȫ���ź�������
 	}
}
   		    


