#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f1xx_hal.h"

void delay_init(unsigned char SYSCLK);
void delay_ms(unsigned short nms);
void delay_us(unsigned int nus);

#endif





























