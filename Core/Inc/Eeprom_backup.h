/*
 * Eeprom_backup.h
 *
 *  Created on: 6 lip 2023
 *      Author: Adrian
 */

#ifndef INC_EEPROM_BACKUP_H_
#define INC_EEPROM_BACKUP_H_

#include "M24Cxx.h"

#define M24_WRITE_TIME 20

enum EepromAddr{
	EEPROM_GPIO_ADDR=0x00,
	EEPROM_PWM1_ADDR=0x02,
	EEPROM_PWM2_ADDR=0x04,
	EEPROM_PWM3_ADDR=0x06,
	EEPROM_PWM4_ADDR=0x08
};

struct EepromMirror{
	uint8_t EepromBufferMirror[255];
	union{
		uint8_t EpromBuffer[255];
		struct{
			uint16_t EepromGpioOut;
			uint16_t EepromPwm1;
			uint16_t EepromPwm2;
			uint16_t EepromPwm3;
			uint16_t EepromPwm4;

			int8_t   EepromLogoMode;
			uint16_t EepromLogoPwmMax;
			uint16_t EepromLogoDimmingSpeed;

			int8_t   EepromLightMode;
			uint16_t EepromLightPwmMax;
			uint16_t EepromLightDimmingSpeed;

			uint32_t EepromPwmFreqPrescaler;
		};
	};

};

extern LedLightParameter_t Logo;
extern LedLightParameter_t Light;



void EepromInit(m24cxx_t *dev);
void EepromRecovery(void);
void EepromBackup(m24cxx_t *dev);
void EepromRefresh(m24cxx_t *dev);


#endif /* INC_EEPROM_BACKUP_H_ */
