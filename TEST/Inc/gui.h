
#ifndef __GUI_H
#define __GUI_H

#include "stm32f1xx_hal.h"

unsigned short LCD_BGR2RGB(unsigned short c);
void Gui_Circle(unsigned short X,unsigned short Y,unsigned short R,unsigned short fc); 
void Gui_DrawLine(unsigned short x0, unsigned short y0,unsigned short x1, unsigned short y1,unsigned short Color);  
void Gui_box(unsigned short x, unsigned short y, unsigned short w, unsigned short h,unsigned short bc);
void Gui_box2(unsigned short x,unsigned short y,unsigned short w,unsigned short h, unsigned char mode);
void DisplayButtonDown(unsigned short x1,unsigned short y1,unsigned short x2,unsigned short y2);
void DisplayButtonUp(unsigned short x1,unsigned short y1,unsigned short x2,unsigned short y2);
void Gui_DrawFont_GBK16(unsigned short x, unsigned short y, unsigned short fc, unsigned short bc, unsigned char *s);
void Gui_DrawFont_GBK24(unsigned short x, unsigned short y, unsigned short fc, unsigned short bc, unsigned char *s);
void Gui_DrawFont_Num32(unsigned short x, unsigned short y, unsigned short fc, unsigned short bc, unsigned short num) ;



#endif
