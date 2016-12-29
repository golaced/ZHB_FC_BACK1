#include "myiic_sonar.h"
#include "delay.h"
#include "time.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/10 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

//��ʼ��IIC
void IIC_Init_Sonar(void)
{					     
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);//ʹ��GPIOBʱ��

  //GPIOB8,B9��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��
	SCL_s=1;
	SDA_s=1;
}


#define uchar unsigned char
#define uint unsigned int
 
u16 sum;    
 
/***************************************************************************
*******************************************************************************/
 u8 IICwriteBytes_S(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start_S();
	IIC_Send_Byte_S(dev);	   //����д����
	IIC_Wait_Ack_S();
	IIC_Send_Byte_S(reg);   //���͵�ַ
    IIC_Wait_Ack_S();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte_S(data[count]); 
		IIC_Wait_Ack_S(); 
	 }
	IIC_Stop_S();//����һ��ֹͣ����

    return 1; //status == 0;
}
 
void Single_WriteI2C(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{       
   IICwriteBytes_S(SlaveAddress, REG_Address, 1, &REG_data);
}          
//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
u8 Single_ReadI2C(u8 SlaveAddress,u8 REG_Address)
{
    u8 REG_data=0;
	unsigned char res=0;
	
	IIC_Start_S();	
	IIC_Send_Byte_S(SlaveAddress);	   //����д����
	res++;
	IIC_Wait_Ack_S();
	IIC_Send_Byte_S(REG_Address); res++;  //���͵�ַ
	IIC_Wait_Ack_S();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start_S();
	IIC_Send_Byte_S(SlaveAddress+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack_S();
	res=IIC_Read_Byte_S(0);	   
    IIC_Stop_S();//����һ��ֹͣ����

	return res;
}
 
u32 Read_KS10X(u8 SlaveAddress)
{    
//       delay_ms(80);
         sum=Single_ReadI2C(SlaveAddress,0x02);             //��8λ
         sum<<=8;
         sum+=Single_ReadI2C(SlaveAddress,0x03);                //��8λ
         return sum;
}
 
void Change_Addr(u8 OldAdddr,u8 NewAddr)
{
//       delay_ms(500);
         Single_WriteI2C(OldAdddr,0x02,0x9a);             //Ĭ��ԭ��ַ��0x00;
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,0x92);
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,0x9e);
         delay_ms(1);
         Single_WriteI2C(OldAdddr,0x02,NewAddr);
         delay_ms(100);
 
//           Single_WriteI2C(NewAddr,0x02,0xb0);
//           delay_ms(80);
}
 
float Read_KS10X_Data(u8 SlaveAddress)
{
        float sumx;
        float sum_x;
        Single_WriteI2C(SlaveAddress,0x02,0xb4);
        Delay_ms(80);
         
        sumx=Read_KS10X(SlaveAddress);          //������һ��������������
       
        sum_x=(float)sumx/10;
//      printf(\"%lf\r\n\",sum_x);
 
        return sum_x;
}
 
void KS10X_Change_Addr_Init(u8 OldAddr,u8 NewAddr)  //�˺�������ʵ��ѡ�������ĵ�ַ
{
        Change_Addr(OldAddr,NewAddr);
        delay_ms(80);
}
 
 

 
//****************************************
// ����KS10X�ڲ���ַ
//****************************************
 
#define SlaveAddress1   0xE8    //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
#define SlaveAddress2   0xd0
#define SlaveAddress3   0xd2
#define SlaveAddress4   0xd4
#define SlaveAddress5   0xd6
#define SlaveAddress6   0xd8
#define SlaveAddress7   0xda
#define SlaveAddress8   0xdc
#define SlaveAddress9   0xde
#define SlaveAddress10  0xe0
#define SlaveAddress11  0xe2
#define SlaveAddress12  0xe4
#define SlaveAddress13  0xe6
#define SlaveAddress14  0xea
#define SlaveAddress15  0xec
#define SlaveAddress16  0xee
#define SlaveAddress17  0xf8
#define SlaveAddress18  0xfa
#define SlaveAddress19  0xfc
#define SlaveAddress20  0xfe
 
void IIC_Start_S(void)
{
	SDA_OUT_s();     //sda�����
	SDA_s=1;	  	  
	SCL_s=1;
	Delay_us(4);
 	SDA_s=0;//START:when CLK is high,DATA change form high to low 
	Delay_us(4);
	SCL_s=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop_S(void)
{
	SDA_OUT_s();//sda�����
	SCL_s=0;
	SDA_s=0;//STOP:when CLK is high DATA change form low to high
 	Delay_us(4);
	SCL_s=1; 
	SDA_s=1;//����I2C���߽����ź�
	Delay_us(4);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack_S(void)
{
	u8 ucErrTime=0;
	SDA_IN_s();      //SDA����Ϊ����  
	SDA_s=1;Delay_us(1);	   
	SCL_s=1;Delay_us(1);	 
	while(READ_SDA_s)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop_S();
			return 1;
		}
	  Delay_us(1);
	}
	SCL_s=0;//ʱ�����0 	   
	return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack_S(void)
{
	SCL_s=0;
	SDA_OUT_s();
	SDA_s=0;
	Delay_us(2);
	SCL_s=1;
	Delay_us(2);
	SCL_s=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck_S(void)
{
	SCL_s=0;
	SDA_OUT_s();
	SDA_s=1;
	Delay_us(2);
	SCL_s=1;
	Delay_us(2);
	SCL_s=0;
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte_S(u8 txd)
{                        
    u8 t;   
	SDA_OUT_s(); 	    
    SCL_s=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        SDA_s=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_us(2);   
		SCL_s=1;
		Delay_us(2); 
		SCL_s=0;	
		Delay_us(2);
    }	 
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte_S(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN_s();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SCL_s=0; 
        Delay_us(2);
		SCL_s=1;
        receive<<=1;
        if(READ_SDA_s)receive++;   
		Delay_us(2); 
    }					 
    if (ack)
        IIC_Ack_S(); //����ACK 
    else
        IIC_NAck_S();//����nACK  
    return receive;
}
 
//int main()
//{
//      Stm32_Clock_Init(9);
//      delay_init(72);
//      uart_init(72,9600);
//      I2C2_Init();
// 
////    KS10X_Change_Addr_Init(SlaveAddress3,SlaveAddress2);  
//      //�ڻ���KS10X���ִ�У���ִ��һ��  ,ִ����һ�κ����ȥ
//      //�������ģ�齫�й̶��ĵ�ַ
// 
//      while(1)
//      {                  
//             sum_2=Read_KS10X_Data(SlaveAddress2);
//             sum_1=Read_KS10X_Data(SlaveAddress1);
//              
//             printf(\"%lf\r\n\",sum_1);
//             printf(\"%lf\r\n\r\n\",sum_2);
//      }
//}

































