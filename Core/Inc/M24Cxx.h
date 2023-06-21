/*
 * M24Cxx.h
 *
 *  Created on: Jun 15, 2023
 *      Author: Adrian
 */

#ifndef INC_M24CXX_H_
#define INC_M24CXX_H_

#include "stm32f1xx.h"

#define M24C01_MEM_SIZE 128
#define M24C02_MEM_SIZE 256
#define M24C04_MEM_SIZE 512
#define M24C08_MEM_SIZE 1024
#define M24C016_MEM_SIZE 2048

typedef struct{

	uint8_t addr;
	I2C_HandleTypeDef *i2c;
	uint16_t memsize;
	GPIO_TypeDef *WcPort;
	uint16_t WcPin;
	uint8_t WcIsZero;


}m24cxx_t;

void m24cxxInit(m24cxx_t *m24, I2C_HandleTypeDef *i2c, uint8_t addr, uint16_t memsize, GPIO_TypeDef *WcPort, uint16_t WcPin);
void m24cxxRead8Bit(m24cxx_t *m24, uint8_t DataAddr, uint8_t *Data);
void m24cxxWrite8Bit(m24cxx_t *m24, uint8_t DataAddr, uint8_t *Data);
void m24cxxRead16Bit(m24cxx_t *m24, uint8_t DataAddr, uint16_t *Data);
void m24cxxWrite16Bit(m24cxx_t *m24, uint8_t DataAddr, uint16_t *Data);
void m24cxxRead32Bit(m24cxx_t *m24, uint8_t DataAddr, uint32_t *Data);
void m24cxxWrite32Bit(m24cxx_t *m24, uint8_t DataAddr, uint32_t *Data);
void m24cxxFullRead(m24cxx_t *m24, uint8_t *Data);
void m24cxxFullWrite(m24cxx_t *m24, uint8_t *Data);
void m24cxxEarese(m24cxx_t *m24);
void m24cxxWcSetIt(m24cxx_t *m24, I2C_HandleTypeDef *hi2c); // Run in TxCplt callback

#endif /* INC_M24CXX_H_ */
