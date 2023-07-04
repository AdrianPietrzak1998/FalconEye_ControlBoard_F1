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


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
