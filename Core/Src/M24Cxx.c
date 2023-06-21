/*
 * M24Cxx.c
 *
 *  Created on: Jun 15, 2023
 *      Author: Adrian
 */
#include "M24Cxx.h"



void m24cxxInit(m24cxx_t *m24, I2C_HandleTypeDef *i2c, uint8_t addr, uint16_t memsize, GPIO_TypeDef *WcPort, uint16_t WcPin)
{
	m24 -> addr    = addr<<1;
	m24 -> i2c     = i2c;
	m24 -> memsize = memsize;
	m24 -> WcPort  = WcPort;
	m24 -> WcPin   = WcPin;
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, SET);
}

void m24cxxRead8Bit(m24cxx_t *m24, uint8_t DataAddr, uint8_t *Data)
{

	HAL_I2C_Mem_Read_DMA(m24 ->i2c, m24 -> addr, DataAddr, 1, Data, 1);
}

void m24cxxWrite8Bit(m24cxx_t *m24, uint8_t DataAddr, uint8_t *Data)
{
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, RESET);
	m24 -> WcIsZero = 1;
	HAL_I2C_Mem_Write_DMA(m24 -> i2c, m24 -> addr, DataAddr, 1, Data, 1);
}

void m24cxxRead16Bit(m24cxx_t *m24, uint8_t DataAddr, uint16_t *Data)
{
	HAL_I2C_Mem_Read_DMA(m24 ->i2c, m24 -> addr, DataAddr, 1, Data, 2);
}

void m24cxxWrite16Bit(m24cxx_t *m24, uint8_t DataAddr, uint16_t *Data)
{
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, RESET);
	m24 -> WcIsZero = 1;
	HAL_I2C_Mem_Write_DMA(m24 -> i2c, m24 -> addr, DataAddr, 1, Data, 2);
}

void m24cxxRead32Bit(m24cxx_t *m24, uint8_t DataAddr, uint32_t *Data)
{
	HAL_I2C_Mem_Read_DMA(m24 ->i2c, m24 -> addr, DataAddr, 1, Data, 4);
}

void m24cxxWrite32Bit(m24cxx_t *m24, uint8_t DataAddr, uint32_t *Data)
{
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, RESET);
	m24 -> WcIsZero = 1;
	HAL_I2C_Mem_Write_DMA(m24 -> i2c, m24 -> addr, DataAddr, 1, Data, 4);
}

void m24cxxFullRead(m24cxx_t *m24, uint8_t *Data)
{
	HAL_I2C_Mem_Read_DMA(m24 -> i2c, m24 -> addr, 0x00, 1, Data, m24 -> memsize);
}

void m24cxxFullWrite(m24cxx_t *m24, uint8_t *Data)
{
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, RESET);
	for(uint16_t memAddr = 0; memAddr<m24 -> memsize; memAddr = memAddr + 16)
	{
		HAL_Delay(3);
		HAL_I2C_Mem_Write(m24 -> i2c, m24 -> addr, memAddr, 1, Data + memAddr, 16, 50);

	}
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, SET);
}

void m24cxxEarese(m24cxx_t *m24)
{
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, RESET);
	uint8_t Fill[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	for(uint16_t memAddr = 0; memAddr<m24 -> memsize; memAddr = memAddr + 8)
		{
		HAL_Delay(3);
		HAL_I2C_Mem_Write(m24 -> i2c, m24 -> addr, memAddr, 1, Fill, 8, 100);
		}
	HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, SET);
}

void m24cxxWcSetIt(m24cxx_t *m24, I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == m24->i2c->Instance && m24->WcIsZero == 1)
	{
		HAL_GPIO_WritePin(m24->WcPort, m24->WcPin, SET);
		m24->WcIsZero = 0;
	}
}