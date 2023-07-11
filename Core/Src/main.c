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
#include "Eeprom_backup.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDRES 0x50
#define V25 1.43F
#define AVG_SLOPE 4.3F

#define DOOR_OPEN !HAL_GPIO_ReadPin(SW_OPEN_GPIO_Port, SW_OPEN_Pin)
#define DOOR_CLOSED HAL_GPIO_ReadPin(SW_OPEN_GPIO_Port, SW_OPEN_Pin)
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

uint32_t OldTick500ms, OldTick100ms, OldTick50ms, OldTick10000ms;

int32_t Temp;
float Temperature;
uint8_t ds1[DS18B20_ROM_CODE_SIZE];

m24cxx_t M24C02;


uint8_t PwmSetPtr = 9;
uint8_t LedParamSetPtr = 5;
LedLightParameter_t *LedLightActualEdit;

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



LedLightParameter_t Logo;
LedLightParameter_t Light;

enum PwmFreqency PwmFrequency;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void UsbBuffWrite(char * Message);
void UsbTransmitTask(void);

void IntervalFunc10000ms(void);
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

void LogoLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);
void LightLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);
void LedLightTask(LedLightParameter_t *Instance);
void LedLightInit(LedLightParameter_t *Instance, TIM_HandleTypeDef *htim, uint8_t Channel, uint16_t DimmerSpeed);

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
void ShowLedLightParam(LedLightParameter_t *Instance);
	void LedLightParamPtrIncrement(void);
	void LedLightParamIncrement(void);
	void LedLightParamDecrement(void);
	void LedLightParamIncrement25(void);
	void LedLightParamDecrement25(void);
	void ShowLedLightParamLogo(void);
	void ShowLedLightParamLight(void);

void PwmFreqSet(uint16_t PwmFrequency);

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
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init(&hi2c1, &hdma_memtomem_dma2_channel1);

  OldTick500ms = HAL_GetTick();
  OldTick100ms = HAL_GetTick();
  OldTick50ms = HAL_GetTick();
  OldTick10000ms = HAL_GetTick();

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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

  LedLightInit(&Logo, &htim3, TIM_CHANNEL_1, 20);
  LedLightInit(&Light, &htim5, TIM_CHANNEL_2, 20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  m24cxxInit(&M24C02, &hi2c1, EEPROM_ADDRES, M24C02_MEM_SIZE, WC_EEPROM_GPIO_Port, WC_EEPROM_Pin);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)Measurements.Adc1Value, 4);

  EepromInit(&M24C02);
  EepromRecovery();



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
	  IntervalFunc10000ms();

	  ButtonTask(&KeyDown);
	  ButtonTask(&KeyUp);

	  LedBlinkTask(&CommPcUsb);

	  MeasurementConversion();

	  LedLightTask(&Logo);
	  LedLightTask(&Light);


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

void PwmFreqSet(uint16_t PwmFrequency)
{
	  htim4.Init.Prescaler = PwmFrequency;
	  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	  {
	    Error_Handler();
	  }
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

void LogoLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed)
{
	Logo.Mode = Mode;
	if(PwmValue != 0) Logo.PwmMax = PwmValue;
	if(DimmerSpeed != 0) Logo.DimmerSpeed = DimmerSpeed;
}

void LightLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed)
{
	Light.Mode = Mode;
	if(PwmValue != 0) Light.PwmMax = PwmValue;
	if(DimmerSpeed != 0)Light.DimmerSpeed = DimmerSpeed;
}

void LedLightTask(LedLightParameter_t *Instance)
{
	switch(Instance -> Mode)
	{
	case -1:
		break;
	case 0:
		__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, 0);
		break;
	case 1:
		if(Instance -> PwmActual > Instance -> PwmMax)
		{
			Instance -> PwmActual = Instance -> PwmMax;
			__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
		}
		if(DOOR_OPEN)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual <= Instance->PwmMax)
			{
				Instance->PwmActual++;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
		}
		else if(DOOR_CLOSED)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual > 0)
			{
				Instance->PwmActual--;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
		}
		break;
	case 2:
		if(Instance -> PwmActual > Instance -> PwmMax)
			{
			Instance -> PwmActual = Instance -> PwmMax;
			__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
			}
		if(DOOR_OPEN)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual > 0)
			{
				Instance->PwmActual--;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
		}
		else if(DOOR_CLOSED)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual < Instance->PwmMax)
			{
				Instance->PwmActual++;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
		}
		break;
	case 3:
		if(!Instance->Direction)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual > 50)
			{
				Instance->PwmActual--;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
			if(Instance->PwmActual <= 50) Instance -> Direction = 1;
		}
		else if(Instance->Direction)
		{
			if(HAL_GetTick() - Instance->LastTick > Instance->DimmerSpeed && Instance->PwmActual <= Instance->PwmMax)
			{
				Instance->PwmActual++;
				__HAL_TIM_SET_COMPARE(Instance->htim, Instance->Channel, Instance->PwmActual);
				Instance->LastTick = HAL_GetTick();
			}
			if(Instance->PwmActual >= Instance->PwmMax) Instance -> Direction = 0;
		}

	}
}

void LedLightInit(LedLightParameter_t *Instance, TIM_HandleTypeDef *htim, uint8_t Channel, uint16_t DimmerSpeed)
{
	Instance -> htim = htim;
	Instance -> Channel = Channel;
	Instance -> DimmerSpeed = DimmerSpeed;
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
	sprintf(buff, "12V: %5.2fV", Measurements.Voltage12);
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

void ShowLedLightParam(LedLightParameter_t *Instance)
{
	char buff[6];
	uint8_t Length;

	ButtonRegisterPressCallback(&KeyDown, LedLightParamPtrIncrement);
	SSD1306_Clear(BLACK);

	GFX_DrawString(0, 0, "PWM", WHITE, 1);
	GFX_DrawString(0, 32, "Speed", WHITE, 1);
	Length = sprintf(buff, "%u", Instance ->PwmMax);
	GFX_DrawString(64-2 - (Length*5), 16, buff, WHITE, 1);
	Length = sprintf(buff, "%u", Instance ->DimmerSpeed);
	GFX_DrawString(64-2 - (Length*5), 48, buff, WHITE, 1);

	if(LedParamSetPtr%2 && LedParamSetPtr!=5)
	{
		ButtonRegisterPressCallback(&KeyUp, LedLightParamDecrement);
		ButtonRegisterRepeatCallback(&KeyUp, LedLightParamDecrement25);
		GFX_DrawChar(25, 16*LedParamSetPtr, '-', WHITE, 1);
	}
	else if(!(LedParamSetPtr%2)&& LedParamSetPtr!=5)
	{
		ButtonRegisterPressCallback(&KeyUp, LedLightParamIncrement);
		ButtonRegisterRepeatCallback(&KeyUp, LedLightParamIncrement25);
		GFX_DrawChar(91, 16*(LedParamSetPtr-1), '+', WHITE, 1);
	}
	else
	{
		ButtonRegisterPressCallback(&KeyUp, ShowMenu);
		GFX_DrawString(0, 47, "<<", WHITE, 1);
	}
}
	void LedLightParamPtrIncrement(void)
	{
		LedParamSetPtr++;
		if(LedParamSetPtr >= 6)
		{
			LedParamSetPtr = 1;
		}
	}
	void LedLightParamIncrement(void)
	{
		switch(LedParamSetPtr/2)
		{
		case 1:
			LedLightActualEdit -> PwmMax++;
			if(LedLightActualEdit -> PwmMax > 1000) LedLightActualEdit->PwmMax = 0;
			break;
		case 2:
			LedLightActualEdit -> DimmerSpeed++;
			if(LedLightActualEdit -> DimmerSpeed >1000) LedLightActualEdit->DimmerSpeed = 0;
			break;
		}
	}
	void LedLightParamIncrement25(void)
	{
		switch(LedParamSetPtr/2)
		{
		case 1:
			LedLightActualEdit -> PwmMax += 25;
			if(LedLightActualEdit -> PwmMax > 1000) LedLightActualEdit->PwmMax = 0;
			break;
		case 2:
			LedLightActualEdit -> DimmerSpeed += 25;
			if(LedLightActualEdit -> DimmerSpeed >1000) LedLightActualEdit->DimmerSpeed = 0;
			break;
		}
	}
	void LedLightParamDecrement(void)
	{
		switch((LedParamSetPtr/2)+1)
		{
		case 1:
			LedLightActualEdit -> PwmMax--;
			if(LedLightActualEdit -> PwmMax > 1000) LedLightActualEdit->PwmMax = 1000;
			break;
		case 2:
			LedLightActualEdit -> DimmerSpeed--;
			if(LedLightActualEdit -> DimmerSpeed > 1000) LedLightActualEdit -> DimmerSpeed = 1000;
		}
	}
	void LedLightParamDecrement25(void)
	{
		switch((LedParamSetPtr/2)+1)
		{
		case 1:
			LedLightActualEdit -> PwmMax -= 25;
			if(LedLightActualEdit -> PwmMax > 1000) LedLightActualEdit->PwmMax = 1000;
			break;
		case 2:
			LedLightActualEdit -> DimmerSpeed -= 25;
			if(LedLightActualEdit -> DimmerSpeed > 1000) LedLightActualEdit -> DimmerSpeed = 1000;
		}
	}
	void ShowLedLightParamLogo(void)
	{
		HideMenu();
		ActualVisibleFunc = ShowLedLightParamLogo;
		LedLightActualEdit = &Logo;
		ShowLedLightParam(LedLightActualEdit);
	}
	void ShowLedLightParamLight(void)
	{
		HideMenu();
		ActualVisibleFunc = ShowLedLightParamLight;
		LedLightActualEdit = &Light;
		ShowLedLightParam(LedLightActualEdit);
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
	Measurements.Voltage12 = Measurements.Voltage12Raw /1241.0F * 5;
	Measurements.Voltage5 = Measurements.Voltage5Raw /1241.0F*2;
	Measurements.InternalTemperature = ((Measurements.InternalTemperatureRaw /1241.0F) - V25) / AVG_SLOPE + 25;
}


void IntervalFunc10000ms(void)
{
	if(HAL_GetTick() - OldTick10000ms >10000)
	{
		char MsgToSend[255];
		EepromRefresh(&M24C02);

		sprintf(MsgToSend, "10/0x%lx%lx%lx",
				HAL_GetUIDw2(),
				HAL_GetUIDw1(),
				HAL_GetUIDw0());
		UsbBuffWrite(MsgToSend);
		OldTick10000ms = HAL_GetTick();
	}
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
									(unsigned int*)((~GPIOG->IDR)&0xff),
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

		sprintf(MsgToSend, "1/%i/%u/%u/%i/%u/%u",
				Light.Mode,
				Light.PwmMax,
				Light.DimmerSpeed,
				Logo.Mode,
				Logo.PwmMax,
				Logo.DimmerSpeed);
		UsbBuffWrite(MsgToSend);
		OldTick100ms = HAL_GetTick();
	}

}

void IntervalFunc50ms(void)
{
	if(HAL_GetTick() - OldTick50ms >50)
	{
		if(ActualVisibleFunc != ShowPWMsetMenu && ActualVisibleFunc != ShowLedLightParamLight && ActualVisibleFunc != ShowLedLightParamLogo)
		{
			EepromBackup(&M24C02);
		}

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
