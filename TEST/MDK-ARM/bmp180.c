

#include "gpio.h"
#include "stdio.h"
#include "math.h"
 
 
//BMP180
uint16_t BMP180_ADDR_W = 0xEE; // ???????
uint16_t BMP180_MADDR_W= 0xF4; //?????
uint16_t BMP180_CMD_WD = 0x2E; //??,????4.5ms
uint16_t BMP180_CMD_QY = 0xF4;  //??,????25.5ms
int osrs = 3;
uint16_t BMP180_ADDR_R = 0xEF; // ?,????? (MSB - ??, LSB ?? )
uint16_t BMP180_MADDR_R = 0xF6;  // ? F6,F7 2?
//BMP???
uint16_t BMP180_AC_BE = 0xAA; //?AA ~ BE
//??BMP?????
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_TypeDef;
BMP180_Calibration_TypeDef BMP180_Calibration;
 
 
 
//485??
void send485(uint8_t *pData,uint8_t len){
	// 485
	//??:DE?0,RE?0,?????
	//??:DE?1,RE? 1,?????
 
	//?????
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); //PB0 = DE = 1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); //PB1 = RE = 1
	//????
	HAL_UART_Transmit(&huart3, pData, len, 1000);
	//?????
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //PB0 = DE = 0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //PB1 = RE = 0
}
 
 
/** I2C???? **/
//????????
void BMP180_Init()
{
	uint8_t buffer[22];
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_AC_BE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer, 22, 3000);
	BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (buffer[10]  << 8) | buffer[11];
	BMP180_Calibration.B1 = (buffer[12]  << 8) | buffer[13];
	BMP180_Calibration.B2 = (buffer[14]  << 8) | buffer[15];
	BMP180_Calibration.MB = (buffer[16]  << 8) | buffer[17];
	BMP180_Calibration.MC = (buffer[18]  << 8) | buffer[19];
	BMP180_Calibration.MD =(buffer[20]  << 8) | buffer[21];
}
//????
int16_t BMP180_Calc_WD(uint16_t UT)
{
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);
	return (BMP180_Calibration.B5 + 8) >> 4;
}
//????
int32_t BMP180_Calc_QY(uint32_t UP, uint8_t oss)
{
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;
 
	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p = p + (((((p >> 8) * (p >> 8) * 3038) >> 16) + ((-7357 * p) >> 16) + 3791) >> 4);
	return p;
}
//????
int32_t BMP180_Calc_HB(int32_t Pa)
{
	return 44330 * (1 - pow( Pa/ 101325.0f, 1.0f / 5.255f));
}
 
//???
void BMP180_Value()
{
	uint8_t wddata[2];
	uint8_t qydata[3];
 
	//????
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR_W, BMP180_MADDR_W, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&BMP180_CMD_WD, 1, 1000);
	HAL_Delay(5);//5ms
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_MADDR_R, I2C_MEMADD_SIZE_8BIT, (uint8_t *)wddata, 2, 1000);
 
	//????
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR_W, BMP180_MADDR_W, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&BMP180_CMD_QY, 1, 1000);
	HAL_Delay(26);//26ms
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR_R, BMP180_MADDR_R, I2C_MEMADD_SIZE_8BIT, (uint8_t *)qydata, 3, 1000);
 
	//???????
	uint16_t UT = (wddata[0]<<8 ) | wddata[1];
	uint16_t temperature = BMP180_Calc_WD(UT);
	uint8_t tem[2];
	tem[0] = (temperature >> 8)&0xff;
	tem[1] = temperature &0xff;
	//???485
	send485((uint8_t *)tem, 2);
	HAL_Delay(200);
	//?????
	uint32_t UP = (qydata[0] << 16) | (qydata[1] <<8) | qydata[2] >> (8 - osrs);
	uint32_t pressure = BMP180_Calc_QY(UP,osrs); //???
	uint8_t pre[4];
	pre[0] = pressure >> 24;
	pre[1] = (pressure >> 16)&0xff;
	pre[2] = (pressure >> 8)&0xff;
	pre[3] = pressure &0xff ;
	send485((uint8_t *)pre, 4);
	HAL_Delay(200);
	//????
	uint32_t high = BMP180_Calc_HB(pressure);
	uint8_t hi[4];
	hi[0] = high >> 24;
	hi[1] = (high >> 16)&0xff;
	hi[2] = (high >> 8)&0xff;
	hi[3] = high &0xff ;
	send485((uint8_t *)hi, 4);
	HAL_Delay(200);
}
 
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
 
  BMP180_Init();
  while (1)
  {
	BMP180_Value();
	HAL_Delay(5000);
  }
}
 