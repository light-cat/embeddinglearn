//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//  功能描述   : 1.44寸LCD 4接口演示例程(STM32系列)
/******************************************************************************
//本程序适用与STM32F103C8
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PA5（SCL）
//              SDA   接PA7（SDA）
//              RES   接PB0
//              DC    接PB1
//              CS    接PA4 
//							BL		接PB10
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
#define GRAY0   0xEF7D   	//灰色0 3165 00110 001011 00101
#define GRAY1   0x8410      	//灰色1      00000 000000 00000
#define GRAY2   0x4208      	//灰色2  1111111111011111




//#define LCD_CTRLA   	  	GPIOA		//定义TFT数据端口
//#define LCD_CTRLB   	  	GPIOB		//定义TFT数据端口



#define LCD_SCL        	GPIO_PIN_5	//PB13--->>TFT --SCL/SCK A
#define LCD_SDA        	GPIO_PIN_7	//PB15 MOSI--->>TFT --SDA/DIN
#define LCD_CS        	GPIO_PIN_4  //MCU_PB11--->>TFT --CS/CE

#define LCD_LED        	GPIO_PIN_10  //MCU_PB9--->>TFT --BL
#define LCD_RS         	GPIO_PIN_1	//PB11--->>TFT --RS/DC
#define LCD_RST     	GPIO_PIN_0	//PB10--->>TFT --RST

//#define LCD_CS_SET(x) LCD_CTRL->ODR=(LCD_CTRL->ODR&~LCD_CS)|(x ? LCD_CS:0)

//液晶控制口置1操作语句宏定义
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



    


//液晶控制口置0操作语句宏定义
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

