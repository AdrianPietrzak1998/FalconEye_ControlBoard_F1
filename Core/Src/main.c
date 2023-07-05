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
#include "led_blink.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRES 0x50
#define V25 1.43F
#define AVG_SLOPE 4.3F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
button_t KeyUp, KeyDown;

blink_t CommPcUsb;

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
uint8_t EpromBufer[255];

uint8_t PwmSetPtr = 9;

volatile uint32_t ITCount;

volatile struct Measurements{

	union{
		uint16_t Adc1Value[4];
		struct
		{
			uint16_t Voltage12Raw;
			uint16_t Voltage5Raw;
			uint16_t CurrentRaw;
			uint16_t InternalTemperatureRaw;
		};
	};
	float Voltage12;
	float Voltage5;
	float Current;
	float InternalTemperature;
}Measurements;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void UsbBuffWrite(char * Message);
void UsbTransmitTask(void);


void IntervalFunc500ms(void);
void IntervalFunc100ms(void);
void IntervalFunc50ms(void);

void MeasurementConversion(void);

void OutputSet(uint16_t ODRvalue);
void OutputPinStateSet(uint8_t Pin, uint8_t State);
void OutputPinToggle(uint8_t Pin);
void PwmSet(uint16_t Pwm1, uint16_t Pwm2, uint16_t Pwm3, uint16_t Pwm4);
void PwmChannelSet(uint8_t Channel, uint16_t Value);
void DisplayContrast(uint8_t Contrast);

void ShowMenu(void);
void HideMenu(void);
void ShowMeasurements(void);
void ShowTemperature(void);
void Show8bitIndicators(uint8_t Data, uint8_t NameNumberStart);
void ShowOut0to7(void);
void ShowOut8to15(void);
void ShowIn0to7(void);
void ShowIn8to15(void);
void ShowPWMsetMenu(void);
	void PwmSetPtrIncrement(void);
	void PwmSetIncrement(void);
	void PwmSetDecrement(void);
	void PwmSetIncrement25(void);
	void PwmSetDecrement25(void);


void all(uint8_t x);

void (*ActualVisibleFunc)(void);

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
  SSD1306_Init(&hi2c1, &hdma_memtomem_dma2_channel1);

  OldTick500ms = HAL_GetTick();
  OldTick100ms = HAL_GetTick();
  OldTick50ms = HAL_GetTick();

  if (ds18b20_read_address(ds1) != HAL_OK)
  {
    Error_Handler();
  }

  ButtonInitKey(&KeyUp, BUTTON_UP_GPIO_Port, BUTTON_UP_Pin, 20, 1000, 350);
  ButtonInitKey(&KeyDown, BUTTON_DOWN_GPIO_Port, BUTTON_DOWN_Pin, 20, 1000, 350);

  LedBlinkInit(&CommPcUsb, COMM_PC_LED_GPIO_Port, COMM_PC_LED_Pin, 20);

  ShowMenu();


  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 100);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 10);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  m24cxxInit(&M24C02, &hi2c1, EEPROM_ADDRES, M24C02_MEM_SIZE, WC_EEPROM_GPIO_Port, WC_EEPROM_Pin);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)Measurements.Adc1Value, 4);


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
			  LedBlinkOne(&CommPcUsb);
			  UsbTransmitTask();

			  DataToTransmit--;
		  }

	  }


	  IntervalFunc100ms();
	  IntervalFunc500ms();
	  IntervalFunc50ms();

	  ButtonTask(&KeyDown);
	  ButtonTask(&KeyUp);

	  LedBlinkTask(&CommPcUsb);

	  MeasurementConversion();




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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}

/* USER CODE BEGIN 4 */

void all(uint8_t x)
{
	HAL_Delay(x);
}

void OutputSet(uint16_t ODRvalue)
{
	GPIOE -> ODR = ODRvalue;
}

void OutputPinStateSet(uint8_t Pin, uint8_t State)
{
	if(State == 1)
	{
		GPIOE -> ODR |= (1 << Pin);
	}
	else if (State == 0)
	{
		GPIOE -> ODR &= ~(1 << Pin);
	}
}

void OutputPinToggle(uint8_t Pin)
{
	GPIOE -> ODR ^= (1 << Pin);
}

void PwmSet(uint16_t Pwm1, uint16_t Pwm2, uint16_t Pwm3, uint16_t Pwm4)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Pwm1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Pwm2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Pwm3);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, Pwm4);
}

void PwmChannelSet(uint8_t Channel, uint16_t Value)
{
	Channel = Channel - 1;

//	uint8_t ChannelMapper[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
//	__HAL_TIM_SET_COMPARE(&htim4, ChannelMapper[Channel], Value);

	__HAL_TIM_SET_COMPARE(&htim4, Channel*4, Value);
}

void DisplayContrast(uint8_t Contrast)
{
	SSD1306_Command(SSD1306_SETCONTRAST);
	SSD1306_Command(Contrast);
}



void ShowMenu(void)
{
	ButtonRegisterPressCallback(&KeyDown, MenuNext);
	ButtonRegisterRepeatCallback(&KeyDown, MenuPrev);
	ButtonRegisterPressCallback(&KeyUp, MenuEnter);
	ButtonRegisterRepeatCallback(&KeyUp, NULL);
	ActualVisibleFunc = ScrollString;
	MenuRefresh();
}

void HideMenu(void)
{
	ButtonRegisterPressCallback(&KeyDown, NULL);
	ButtonRegisterRepeatCallback(&KeyDown, NULL);
	ButtonRegisterPressCallback(&KeyUp, NULL);
	ActualVisibleFunc = NULL;
}

void ShowMeasurements(void)
{
	HideMenu();
	ActualVisibleFunc = ShowMeasurements;
	ButtonRegisterPressCallback(&KeyDown, ShowMenu);
	SSD1306_Clear(BLACK);
	char buff[16];
	sprintf(buff, "5V:   %.2fV", Measurements.Voltage5);
	GFX_DrawString(0, 0, buff, WHITE, 1);
	sprintf(buff, "12V:  %.2fV", Measurements.Voltage12);
	GFX_DrawString(0, 16, buff, WHITE, 1);
	sprintf(buff, "Curr: %.2fA", Measurements.Current);
	GFX_DrawString(0, 32, buff, WHITE, 1);
}

void ShowTemperature(void)
{
	HideMenu();
	ActualVisibleFunc = ShowTemperature;
	ButtonRegisterPressCallback(&KeyDown, ShowMenu);
	SSD1306_Clear(BLACK);
	char buff[16];
	sprintf(buff, "MCU: %.2fC", Measurements.InternalTemperature);
	GFX_DrawString(0, 0, buff, WHITE, 1);
	sprintf(buff, "Amb: %.2fC", Temperature);
	GFX_DrawString(0, 16, buff, WHITE, 1);
}

void Show8bitIndicators(uint8_t Data, uint8_t NameNumberStart)
{
	ButtonRegisterPressCallback(&KeyDown, ShowMenu);
	SSD1306_Clear(BLACK);

	char buff[4];
	GFX_SetFont(font_8x5);
	GFX_SetFontSize(1);
	for(uint8_t i=0; i<=7; i++)
	{
		sprintf(buff, "%u.", NameNumberStart + i);
		uint8_t y = 20;
		uint8_t y_name = 0;
		if(i>3)
		{
			y = 43;
			y_name = 57;
		}
		GFX_DrawString(15 + (30 * ((i<4)?i:i-4)), y_name, buff, WHITE, 1);
		if((Data>>i)&1)
		{
			GFX_DrawFillCircle(20 + (30 * ((i<4)?i:i-4)), y, 10, WHITE);
		}
		else
		{
			GFX_DrawCircle(20 + (30 * ((i<4)?i:i-4)), y, 10, WHITE);
		}


	}
}

void ShowOut0to7(void)
{
	HideMenu();
	ActualVisibleFunc = ShowOut0to7;
	Show8bitIndicators(GPIOE -> ODR, 0);
}

void ShowOut8to15(void)
{
	HideMenu();
	ActualVisibleFunc = ShowOut8to15;
	Show8bitIndicators((GPIOE -> ODR) >> 8, 8);
}

void ShowIn0to7(void)
{
	HideMenu();
	ActualVisibleFunc = ShowIn0to7;
	Show8bitIndicators(~(GPIOG -> IDR), 0);
}

void ShowIn8to15(void)
{
	HideMenu();
	ActualVisibleFunc = ShowIn8to15;
	Show8bitIndicators(~((GPIOG -> IDR) >> 8), 8);
}

void ShowPWMsetMenu(void)
{
	char buff[6];
	uint8_t Length;
	HideMenu();
	ActualVisibleFunc = ShowPWMsetMenu;
	ButtonRegisterPressCallback(&KeyDown, PwmSetPtrIncrement);
	SSD1306_Clear(BLACK);

	for(uint8_t i = 0; i<4; i++)
	{
		Length = sprintf(buff, "%u", __HAL_TIM_GET_COMPARE(&htim4, i * 4));
		GFX_DrawString(64-2 - (Length*5), 16*i, buff, WHITE, 1);
	}

	if(PwmSetPtr%2 && PwmSetPtr!=9)
	{
		ButtonRegisterPressCallback(&KeyUp, PwmSetDecrement);
		ButtonRegisterRepeatCallback(&KeyUp, PwmSetDecrement25);
		GFX_DrawChar(25, 16*(PwmSetPtr/2), '-', WHITE, 1);
	}
	else if(!(PwmSetPtr%2))
	{
		ButtonRegisterPressCallback(&KeyUp, PwmSetIncrement);
		ButtonRegisterRepeatCallback(&KeyUp, PwmSetIncrement25);
		GFX_DrawChar(91, 16*((PwmSetPtr/2)-1), '+', WHITE, 1);
	}
	else
	{
		ButtonRegisterPressCallback(&KeyUp, ShowMenu);
		GFX_DrawString(0, 47, "<<", WHITE, 1);
	}
}

	void PwmSetPtrIncrement(void)
	{
		PwmSetPtr++;
		if(PwmSetPtr>=10)PwmSetPtr=1;
	}
	void PwmSetIncrement(void)
	{
		uint8_t CurrentPwmChannel = PwmSetPtr/2;
		uint16_t PwmValue = __HAL_TIM_GET_COMPARE(&htim4, (CurrentPwmChannel - 1) * 4);

		PwmValue += 1;
		if(PwmValue > 1000) PwmValue = 0;
		PwmChannelSet(CurrentPwmChannel, PwmValue);
	}
	void PwmSetDecrement(void)
	{
		uint8_t CurrentPwmChannel = PwmSetPtr/2 + 1;
		uint16_t PwmValue = __HAL_TIM_GET_COMPARE(&htim4, (CurrentPwmChannel - 1)*4);

		PwmValue -= 1;
		if(PwmValue > 1000) PwmValue = 1000;
		PwmChannelSet(CurrentPwmChannel, PwmValue);
	}
	void PwmSetIncrement25(void)
	{
		uint8_t CurrentPwmChannel = PwmSetPtr/2;
		uint16_t PwmValue = __HAL_TIM_GET_COMPARE(&htim4, (CurrentPwmChannel - 1) * 4);

		PwmValue += 25;
		if(PwmValue > 1000) PwmValue = 0;
		PwmChannelSet(CurrentPwmChannel, PwmValue);
	}
	void PwmSetDecrement25(void)
	{
		uint8_t CurrentPwmChannel = PwmSetPtr/2 + 1;
		uint16_t PwmValue = __HAL_TIM_GET_COMPARE(&htim4, (CurrentPwmChannel - 1)*4);

		PwmValue -= 25;
		if(PwmValue > 1000) PwmValue = 1000;
		PwmChannelSet(CurrentPwmChannel, PwmValue);
	}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc ->Instance == ADC1)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Measurements.Adc1Value, 4);
		ITCount++;
	}
}

void MeasurementConversion(void)
{
	Measurements.Voltage5 = Measurements.Voltage5Raw /1241.0F*2;
	Measurements.InternalTemperature = ((Measurements.InternalTemperatureRaw /1241.0F) - V25) / AVG_SLOPE + 25;
}



void IntervalFunc500ms(void)
{
	  if(HAL_GetTick() - OldTick500ms >500)
	  {
		  OldTick500ms = HAL_GetTick();

//		  if(M24C02.i2c -> State == HAL_I2C_STATE_READY)
//		  {
//			  m24cxxFullRead(&M24C02, EpromBufer);
//		  }



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
		char MsgToSend[255];
		/*
		 * Message to send id. 0.
		 * 0/Input 16bit/Output 16bit/PWM1/PWM2/PWM3/PWM4/Temperature/12V/5V/Current
		 */
		sprintf(MsgToSend, "0/%u/%u/%u/%u/%u/%u/%.2f/%.2f/%.2f/%.2f/%.2f",
									(uint16_t*)((~GPIOG->IDR)&0xff),
									(uint16_t*)GPIOE->ODR,
									(uint16_t*)__HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_1),
									(uint16_t*)__HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_2),
									(uint16_t*)__HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_3),
									(uint16_t*)__HAL_TIM_GetCompare(&htim4, TIM_CHANNEL_4),
									Temperature,
									Measurements.InternalTemperature,
									Measurements.Voltage12,
									Measurements.Voltage5,
									Measurements.Current);
		UsbBuffWrite(MsgToSend);
		OldTick100ms = HAL_GetTick();
	}

}

void IntervalFunc50ms(void)
{
	if(HAL_GetTick() - OldTick50ms >50)
	{
		if(ActualVisibleFunc != NULL)
		{
			ActualVisibleFunc();
		}

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
		LedBlinkOne(&CommPcUsb); //Control Led
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
