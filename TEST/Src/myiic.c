#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE(); // 时钟使能
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7; // 模拟IO的引脚
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 设置速率为50MHZ
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);   
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	   // 第一步，将SDA,SCL线全部拉高  
	IIC_SCL=1;   
	delay_us(4);    //第二步，拉高后延迟4us
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low  // 第三步 在SCL高电平时，拉低SDA信号
	delay_us(4);    //延时一段时间
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据  SCL低电平 ，不采样数据，此时允许数据变化
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;   // SCL设置为低电平
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; // 高电平时由低到高
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char IIC_Wait_Ack(void)
{
	unsigned char ucErrTime=0;
	SDA_IN();      //SDA设置为输入   要接受应答信号，配置的为上拉下拉输入模式
	IIC_SDA=1;delay_us(1);	   // 上拉 
	IIC_SCL=1;delay_us(1);	 // 所谓的上拉
	while(READ_SDA)    // 读取是否为零，为零就意味着被拉下了 在下一个时钟沿到来之前，要将电平下拉 表示应答
						// 从机向主机发送应答信号
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)    // 主机对从机产生应答信号  //在一个时钟周期
{
	IIC_SCL=0; 
	SDA_OUT(); 
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(unsigned char txd)
{                        
    unsigned char t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)   // 经典做法，先与后移位 
			IIC_SDA=1;      // 输出1 
		else
			IIC_SDA=0;  
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



























