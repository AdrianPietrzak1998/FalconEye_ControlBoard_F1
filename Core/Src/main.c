/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "M24Cxx.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "ring_buffer.h"
#include "string.h"
#include "utils.h"
#include "parser_complex.h"
#include "ssd1306.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
#include "button.h"
#include "menu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRES 0x50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
button_t KeyUp, KeyDown;
RingBuffer_t ReceiveBuffer;
RingBuffer_t TransmitBuffer;

uint8_t LineCounter;
uint8_t ReceivedData[RING_BUFFER_SIZE];
uint8_t TransmitData[RING_BUFFER_SIZE];

uint8_t DataToTransmit;

extern USBD_HandleTypeDef hUsbDeviceFS;

uint32_t OldTick500ms, OldTick100ms, OldTick50ms;

int32_t Temp;
float Temperature;
uint8_t ds1[DS18B20_ROM_CODE_SIZE];

m24cxx_t M24C02;
uint8_t EpromBufer[256];

uint8_t buff[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void UsbBuffWrite(char * Message);
void UsbTransmitTask(void);

void GpioELedToggle(void);
void GpioFLedToggle(void);

void IntervalFunc500ms(void);
void IntervalFunc100ms(void);
void IntervalFunc50ms(void);


void all(uint8_t x);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	for(uint8_t i = 0; i<255; i++)
	{
		buff[i] = 'a';
	}
	buff[254] = 0;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  OldTick500ms = HAL_GetTick();
  OldTick100ms = HAL_GetTick();
  OldTick50ms = HAL_GetTick();

  if (ds18b20_read_address(ds1) != HAL_OK)
  {
    Error_Handler();
  }

  ButtonInitKey(&KeyUp, BUTTON_UP_GPIO_Port, BUTTON_UP_Pin, 20, 1000, 500);
  ButtonInitKey(&KeyDown, BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin, 20, 1000, 500);

  ButtonRegisterPressCallback(&KeyDown, MenuNext);
  ButtonRegisterRepeatCallback(&KeyDown, MenuPrev);
  ButtonRegisterPressCallback(&KeyUp, MenuEnter);

  SSD1306_Init(&hi2c1);

  MenuRefresh();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIOF->ODR = 0x400;

  m24cxxInit(&M24C02, &hi2c1, EEPROM_ADDRES, M24C02_MEM_SIZE, WC_EEPROM_GPIO_Port, WC_EEPROM_Pin);

  while (1)
  {
	  if(LineCounter)
	  {
		  Parser_TakeLine(&ReceiveBuffer, ReceivedData);

		  LineCounter--;

		  Parser_parse(ReceivedData);
	  }
	  if(DataToTransmit > 0 )
	  {

		  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
		  if (hcdc->TxState == 0)
		  {
			  UsbTransmitTask();

			  DataToTransmit--;
		  }

	  }

	  IntervalFunc100ms();
	  IntervalFunc500ms();
	  IntervalFunc50ms();

	  ButtonTask(&KeyDown);
	  ButtonTask(&KeyUp);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* USB_LP_CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/* USER CODE BEGIN 4 */

void all(uint8_t x)
{
	HAL_Delay(x);
}



void GpioFLedToggle()
{
	GPIOF->ODR = GPIOF->ODR << 1;
	if(!(GPIOF->ODR <= 0x8000 && GPIOF->ODR >= 0x200))
	{
		GPIOF->ODR = 0x200;
	}
}

void IntervalFunc500ms(void)
{
	  if(HAL_GetTick() - OldTick500ms >500)
	  {
		  OldTick500ms = HAL_GetTick();

		  if(M24C02.i2c -> State == HAL_I2C_STATE_READY)
		  {
			  m24cxxFullRead(&M24C02, EpromBufer);
		  }

//		  GpioELedToggle();
		  GpioFLedToggle();
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

		  static uint8_t TempMeasureFlag = 0;
		  if(!TempMeasureFlag)
		  {
			  ds18b20_start_measure(NULL);
			  TempMeasureFlag = 1;
		  }
		  else
		  {
			  Temp = ds18b20_get_temp_wo_fp(NULL);
			  Temperature = Temp;
			  Temperature = Temperature/100;
			  TempMeasureFlag = 0;
		  }
	  }
}

void IntervalFunc100ms(void)
{
	if(HAL_GetTick() - OldTick100ms >100)
	{
		UsbBuffWrite((char*)buff);
		OldTick100ms = HAL_GetTick();
	}

}

void IntervalFunc50ms(void)
{
	if(HAL_GetTick() - OldTick50ms >50)
	{
		ScrollString();
		SSD1306_Display();

		OldTick50ms = HAL_GetTick();
	}

}



void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	m24cxxWcSetIt(&M24C02, hi2c);
}

void CDC_ReveiveCallback(uint8_t *Buffer, uint8_t Length)
{
	if(Length > 0)
	{
		uint8_t i = 0;
		while(i < Length)
		{
		if (RB_OK == Ring_Buffer_Write(&ReceiveBuffer, Buffer[i]))
		{
			if(Buffer[i] == ENDLINE)
			{
				LineCounter++;
			}
		}
		else
		{
			Ring_Buffer_Flush(&ReceiveBuffer);
			UsbBuffWrite("ERROR");
		}
		i++;
		}
	}
}

void UsbBuffWrite(char * Message)
{

	DataToTransmit++;
	for(uint8_t y = 0 ; y < strlen(Message) ; y++)
	{
		if(RB_OK == Ring_Buffer_Write(&TransmitBuffer, Message[y]))
		{

		}
		else
		{
			Ring_Buffer_Flush(&TransmitBuffer);
			DataToTransmit--;
		}
	}

	if(RB_OK == Ring_Buffer_Write(&TransmitBuffer, '^'))
	{

	}
	else
	{
		Ring_Buffer_Flush(&TransmitBuffer);
		DataToTransmit--;
	}

}

void UsbTransmitTask(void)
{
	uint8_t i = 0;
	uint8_t tmp = 0;
		do
		{
			if(Ring_Buffer_Read(&TransmitBuffer, &tmp) == RB_OK)
			{
			TransmitData[i] = tmp;
			i++;
			}
			else
			{
				break;
			}

		} while(tmp != '^');

		CDC_Transmit_FS(TransmitData, i);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
