/*
 * Eeprom_backup.c
 *
 *  Created on: 6 lip 2023
 *      Author: Adrian
 */
#include "main.h"
#include "Eeprom_backup.h"
#include "tim.h"

enum EepromAddr EepromAddr;
struct EepromMirror EepromMirror;

uint32_t OldTickReadEeprom;

void EepromInit(m24cxx_t *dev)
{
	m24cxxFullReadWoDma(dev, EepromMirror.EepromBufferMirror);
	for(uint16_t i = 0; i<dev->memsize; i++ )
	{
		EepromMirror.EpromBuffer[i] = EepromMirror.EepromBufferMirror[i];
	}
}

void EepromRecovery(void)
{
	GPIOE -> ODR = EepromMirror.EepromGpioOut;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, EepromMirror.EepromPwm1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, EepromMirror.EepromPwm2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, EepromMirror.EepromPwm3);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, EepromMirror.EepromPwm4);
}

static void EepromVarAssig(void)
{
	EepromMirror.EepromGpioOut = GPIOE -> ODR;
	EepromMirror.EepromPwm1 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
	EepromMirror.EepromPwm2 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
	EepromMirror.EepromPwm3 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3);
	EepromMirror.EepromPwm4 = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4);
}

void EepromBackup(m24cxx_t *dev)
{
	EepromVarAssig();

	for(uint16_t i = 0; i<dev->memsize; i++)
	{
		if(EepromMirror.EepromBufferMirror[i] != EepromMirror.EpromBuffer[i])
		{
			m24cxxWrite8Bit(dev, i, &EepromMirror.EpromBuffer[i]);
//			EepromMirror.EepromBufferMirror[i] = EepromMirror.EpromBuffer[i];
			m24cxxRead8Bit(dev, i, &EepromMirror.EepromBufferMirror[i]);
			break;
		}
	}
}

void EepromRefresh(m24cxx_t *dev)
{
	m24cxxFullRead(dev, EepromMirror.EepromBufferMirror);
}
