#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE(); // ʱ��ʹ��
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; // ģ��IO������
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // ��©���
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // ��������Ϊ50MHZ
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);   
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	   // ��һ������SDA,SCL��ȫ������  
	IIC_SCL=1;   
	delay_us(4);    //�ڶ��������ߺ��ӳ�4us
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low  // ������ ��SCL�ߵ�ƽʱ������SDA�ź�
	delay_us(4);    //��ʱһ��ʱ��
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������  SCL�͵�ƽ �����������ݣ���ʱ�������ݱ仯
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;   // SCL����Ϊ�͵�ƽ
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; // �ߵ�ƽʱ�ɵ͵���
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
unsigned char IIC_Wait_Ack(void)
{
	unsigned char ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����   Ҫ����Ӧ���źţ����õ�Ϊ������������ģʽ
	IIC_SDA=1;delay_us(1);	   // ���� 
	IIC_SCL=1;delay_us(1);	 // ��ν������
	while(READ_SDA)    // ��ȡ�Ƿ�Ϊ�㣬Ϊ�����ζ�ű������� ����һ��ʱ���ص���֮ǰ��Ҫ����ƽ���� ��ʾӦ��
						// �ӻ�����������Ӧ���ź�
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)    // �����Դӻ�����Ӧ���ź�  //��һ��ʱ������
{
	IIC_SCL=0; 
	SDA_OUT(); 
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(unsigned char txd)
{                        
    unsigned char t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)   // �����������������λ 
			IIC_SDA=1;      // ���1 
		else
			IIC_SDA=0;  
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























