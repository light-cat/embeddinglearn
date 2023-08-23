//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//  ��������   : 1.44��LCD 4�ӿ���ʾ����(STM32ϵ��)
/******************************************************************************
//������������STM32F103C8
//              GND   ��Դ��
//              VCC   ��5V��3.3v��Դ
//              SCL   ��PA5��SCL��
//              SDA   ��PA7��SDA��
//              RES   ��PB0
//              DC    ��PB1
//              CS    ��PA4 
//							BL		��PB10
*******************************************************************************/
//******************************************************************************/
#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H
#include "stm32f1xx_hal.h"

#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111




//#define LCD_CTRLA   	  	GPIOA		//����TFT���ݶ˿�
//#define LCD_CTRLB   	  	GPIOB		//����TFT���ݶ˿�



#define LCD_SCL        	GPIO_PIN_5	//PB13--->>TFT --SCL/SCK A
#define LCD_SDA        	GPIO_PIN_7	//PB15 MOSI--->>TFT --SDA/DIN
#define LCD_CS        	GPIO_PIN_4  //MCU_PB11--->>TFT --CS/CE

#define LCD_LED        	GPIO_PIN_10  //MCU_PB9--->>TFT --BL
#define LCD_RS         	GPIO_PIN_1	//PB11--->>TFT --RS/DC
#define LCD_RST     	GPIO_PIN_0	//PB10--->>TFT --RST

//#define LCD_CS_SET(x) LCD_CTRL->ODR=(LCD_CTRL->ODR&~LCD_CS)|(x ? LCD_CS:0)

//Һ�����ƿ���1�������궨��
//#define	LCD_SCL_SET  	LCD_CTRLA->BSRR=LCD_SCL 
//#define	LCD_SDA_SET  	LCD_CTRLA->BSRR=LCD_SDA   
//#define	LCD_CS_SET  	LCD_CTRLA->BSRR=LCD_CS  
//#define	LCD_LED_SET  	LCD_CTRLB->BSRR=LCD_LED   
//#define	LCD_RS_SET  	LCD_CTRLB->BSRR=LCD_RS 
//#define	LCD_RST_SET  	LCD_CTRLB->BSRR=LCD_RST

#define LCD_SCL_SET  HAL_GPIO_WritePin(GPIOA,LCD_SCL,GPIO_PIN_SET)
#define LCD_SDA_SET  HAL_GPIO_WritePin(GPIOA,LCD_SDA,GPIO_PIN_SET)
#define LCD_CS_SET  HAL_GPIO_WritePin(GPIOA,LCD_CS,GPIO_PIN_SET)

#define LCD_LED_SET  HAL_GPIO_WritePin(GPIOB,LCD_LED,GPIO_PIN_SET)
#define LCD_RS_SET  HAL_GPIO_WritePin(GPIOB,LCD_RS,GPIO_PIN_SET)
#define LCD_RST_SET  HAL_GPIO_WritePin(GPIOB,LCD_RST,GPIO_PIN_SET)



    


//Һ�����ƿ���0�������궨��
#define LCD_SCL_CLR  HAL_GPIO_WritePin(GPIOA,LCD_SCL,GPIO_PIN_RESET)
#define LCD_SDA_CLR  HAL_GPIO_WritePin(GPIOA,LCD_SDA,GPIO_PIN_RESET)
#define LCD_CS_CLR HAL_GPIO_WritePin(GPIOA,LCD_CS,GPIO_PIN_RESET)

#define LCD_LED_CLR  HAL_GPIO_WritePin(GPIOB,LCD_LED,GPIO_PIN_RESET)
#define LCD_RS_CLR  HAL_GPIO_WritePin(GPIOB,LCD_RS,GPIO_PIN_RESET)
#define LCD_RST_CLR  HAL_GPIO_WritePin(GPIOB,LCD_RST,GPIO_PIN_RESET)

//#define	LCD_SCL_CLR  	LCD_CTRLA->BRR=LCD_SCL  
//#define	LCD_SDA_CLR  	LCD_CTRLA->BRR=LCD_SDA 
//#define	LCD_CS_CLR  	LCD_CTRLA->BRR=LCD_CS 
    
//#define	LCD_LED_CLR  	LCD_CTRLB->BRR=LCD_LED 
//#define	LCD_RST_CLR  	LCD_CTRLB->BRR=LCD_RST
//#define	LCD_RS_CLR  	LCD_CTRLB->BRR=LCD_RS 





#define LCD_WR_DATA(data){\
LCD_RS_SET;\
LCD_CS_CLR;\
LCD_DATAOUT(data);\
LCD_WR_CLR;\
LCD_WR_SET;\
LCD_CS_SET;\
} 



void LCD_GPIO_Init(void);
void Lcd_WriteIndex(unsigned char Index);
void Lcd_WriteData(unsigned char Data);
void Lcd_WriteReg(unsigned char Index,unsigned char Data);
unsigned short Lcd_ReadReg(unsigned char LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(unsigned short Color);
void Lcd_SetXY(unsigned short x,unsigned short y);
void Gui_DrawPoint(unsigned short x,unsigned short y,unsigned short Data);
unsigned int Lcd_ReadPoint(unsigned short x,unsigned short y);
void Lcd_SetRegion(unsigned short x_start,unsigned short y_start,unsigned short x_end,unsigned short y_end);
void LCD_WriteData_16Bit(unsigned short Data);

#endif 

