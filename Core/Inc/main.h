/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	int8_t Mode;
	uint16_t PwmMax;
	TIM_HandleTypeDef *htim;
	uint8_t Channel;
	uint16_t PwmActual;
	uint32_t LastTick;
	uint16_t DimmerSpeed;

	uint8_t Direction;
}LedLightParameter_t;

enum PwmFreqency{
	HZ60 = 1199,
	HZ100 = 719,
	HZ200 = 359,
	HZ300 = 239
};

volatile struct ErrorCode{
	union{
		uint16_t Error;
		struct{
			uint16_t DsError : 1;
			uint16_t OledError : 1;
			uint16_t EepromError : 1;
		};
	};
};

extern volatile struct ErrorCode ErrorCode;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define __LOGO_LED_SET(x) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, x)
#define __LOGO_LED_GET __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1)
#define __LIGHT_LED_SET(x) __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, x)
#define __LIGHT_LED_GET __HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define COMM_PC_LED_Pin GPIO_PIN_10
#define COMM_PC_LED_GPIO_Port GPIOF
#define CHECK_WIRE_1_Pin GPIO_PIN_0
#define CHECK_WIRE_1_GPIO_Port GPIOC
#define CHECK_WIRE_2_Pin GPIO_PIN_1
#define CHECK_WIRE_2_GPIO_Port GPIOC
#define CHECK_WIRE_3_Pin GPIO_PIN_2
#define CHECK_WIRE_3_GPIO_Port GPIOC
#define CHECK_WIRE_4_Pin GPIO_PIN_3
#define CHECK_WIRE_4_GPIO_Port GPIOC
#define SW_OPEN_Pin GPIO_PIN_0
#define SW_OPEN_GPIO_Port GPIOA
#define RS485_TXE_Pin GPIO_PIN_4
#define RS485_TXE_GPIO_Port GPIOA
#define COMM_CAN_LED_Pin GPIO_PIN_11
#define COMM_CAN_LED_GPIO_Port GPIOF
#define COMM_CAN_CHECK_LED_Pin GPIO_PIN_12
#define COMM_CAN_CHECK_LED_GPIO_Port GPIOF
#define COMM_UART_LED_Pin GPIO_PIN_13
#define COMM_UART_LED_GPIO_Port GPIOF
#define COMM_RS485_LED_Pin GPIO_PIN_14
#define COMM_RS485_LED_GPIO_Port GPIOF
#define ERROR_LED_Pin GPIO_PIN_15
#define ERROR_LED_GPIO_Port GPIOF
#define BUTTON_DOWN_Pin GPIO_PIN_8
#define BUTTON_DOWN_GPIO_Port GPIOC
#define BUTTON_UP_Pin GPIO_PIN_9
#define BUTTON_UP_GPIO_Port GPIOC
#define WC_EEPROM_Pin GPIO_PIN_5
#define WC_EEPROM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ERROR_DS ErrorCode.DsError = 1
#define ERROR_OLED ErrorCode.OledError = 1
#define ERROR_EEPROM ErrorCode.EepromError = 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
