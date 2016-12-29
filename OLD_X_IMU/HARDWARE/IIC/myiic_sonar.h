#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
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


   	   		   
#define SDA_IN_s()  {GPIOE->MODER&=~(3<<(7*2));GPIOE->MODER|=0<<7*2;}	//PB9����ģʽ
#define SDA_OUT_s() {GPIOE->MODER&=~(3<<(7*2));GPIOE->MODER|=1<<7*2;} //PB9���ģʽ


//IO��������	 
#define SCL_s    PEout(6) //SCL
#define SDA_s    PEout(7) //SDA	 
#define READ_SDA_s   PEin(7)  //����SDA 

//IIC���в�������
void IIC_Init_Sonar(void);                //��ʼ��IIC��IO��				 
void IIC_Start_Sonar(void);				//����IIC��ʼ�ź�
void IIC_Stop_Sonar(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte_Sonar(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte_Sonar(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_Sonar(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack_Sonar(void);					//IIC����ACK�ź�
void IIC_NAck_Sonar(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte_Sonar(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte_Sonar(u8 daddr,u8 addr);	  


#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	  8191
#define AT24C128	16383
#define AT24C256	32767  
//Mini STM32������ʹ�õ���24c02�����Զ���EE_TYPEΪAT24C02
#define EE_TYPE AT24C02
					  
u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//ָ����ַ��ȡһ���ֽ�

u8 KS103_ReadOneByte(u8 address, u8 reg);

void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//ָ����ַд��һ���ֽ�

void KS103_WriteOneByte(u8 address,u8 reg,u8 command);

void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//ָ����ַ��ʼд��ָ�����ȵ�����
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//��ָ����ַ��ʼд��ָ�����ȵ�����
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����


u8 AT24CXX_Check(void);  //�������

u8 Single_ReadI2C(u8 SlaveAddress,u8 REG_Address);
void Single_WriteI2C(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u32 Read_KS10X(u8 SlaveAddress);
void Change_Addr(u8 OldAdddr,u8 NewAddr);
float Read_KS10X_Data(u8 SlaveAddress);
void KS10X_Change_Addr_Init(u8 OldAddr,u8 NewAddr);

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
 void IIC_Start_S(void);               //����IIC��ʼ�ź�
void IIC_Stop_S(void);                //����IICֹͣ�ź�
void IIC_Send_Byte_S(u8 txd);         //IIC����һ���ֽ�
u8 IIC_Read_Byte_S(unsigned char ack);                 //IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack_S(void);              //IIC�ȴ�ACK�ź�
void IIC_Ack_S(void);                 //IIC����ACK�ź�
void IIC_NAck_S(void);                //IIC������ACK�ź�

#endif
















