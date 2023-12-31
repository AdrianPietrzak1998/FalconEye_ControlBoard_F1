/*
 * button.c
 *
 *  Created on: May 7, 2022
 *      Author: Adrian
 */

#include "main.h"
#include "button.h"

//Button init
void ButtonInitKey(button_t * Key, GPIO_TypeDef *GpioPort, uint16_t GpioPin, uint32_t TimerDebounce, uint32_t TimerLongPressed, uint32_t TimerRepeat)
{
	Key->State = IDLE;
	Key->GpioPort = GpioPort;
	Key->GpioPin = GpioPin;
	Key->TimerDebounce = TimerDebounce;
	Key->TimerLongPressed = TimerLongPressed;
	Key->TimerRepeat = TimerRepeat;
}
//Time settings function
void ButtonSetDebounceTime(button_t * Key, uint32_t Miliseconds)
{
	Key->TimerDebounce = Miliseconds;
}

void ButtonSetLongPressedTime(button_t *Key, uint32_t Miliseconds)
{
	Key->TimerLongPressed = Miliseconds;
}

void ButtonSetRepeatTime(button_t *Key, uint32_t Miliseconds)
{
	Key->TimerRepeat = Miliseconds;
}

//Callbacks
void ButtonRegisterPressCallback(button_t *Key, void *Callback)
{
	Key->ButtonPressed = Callback;
}

void ButtonRegisterLongPressedCallback(button_t *Key, void *Callback)
{
	Key->ButtonLongPressed = Callback;
}

void ButtonRegisterRepeatCallback(button_t *Key, void *Callback)
{
	Key->ButtonRepeat = Callback;
}

void ButtonRegisterReleaseCallback(button_t *Key, void *Callback)
{
	Key->ButtonRelease = Callback;
}
//States routine
void ButtonIdleRoutine(button_t *Key)
{
	if(HAL_GPIO_ReadPin(Key->GpioPort, Key->GpioPin) == GPIO_PIN_RESET)
	{
		Key->LastTick = HAL_GetTick();
		Key->State = DEBOUNCE;
	}
}

void ButtonDebounceRoutine(button_t *Key)
{
	if((HAL_GetTick() - Key->LastTick) >= Key->TimerDebounce)
	{
		if(HAL_GPIO_ReadPin(Key->GpioPort, Key->GpioPin) == GPIO_PIN_RESET)
		{
			Key->State = PRESSED;
			Key->LastTick = HAL_GetTick();
			if(Key->ButtonPressed != NULL)
			{
				Key->ButtonPressed();
			}
		}
		else
		{
			Key->State = IDLE;
		}
	}
}

void ButtonPressedRoutine(button_t *Key)
{
	if(HAL_GPIO_ReadPin(Key->GpioPort, Key->GpioPin) == GPIO_PIN_SET)
	{
		Key->State = RELEASE;
	}
	else if(HAL_GetTick() - Key->LastTick >= Key->TimerLongPressed)
	{
		Key->State = REPEAT;
		Key->LastTick = HAL_GetTick();
		if(Key->ButtonLongPressed != NULL)
		{
			Key->ButtonLongPressed();
		}
	}
}

void ButtonRepeatRoutine(button_t *Key)
{
	if(HAL_GPIO_ReadPin(Key->GpioPort, Key->GpioPin) == GPIO_PIN_SET)
	{
		Key->State = RELEASE;
	}
	else if(HAL_GetTick() - Key->LastTick >= Key->TimerRepeat)
	{
		Key->LastTick = HAL_GetTick();
		if(Key->ButtonRepeat != NULL)
		{
			Key->ButtonRepeat();
		}
	}
}

void ButtonReleaseRoutine(button_t *Key)
{
	if(Key->ButtonRelease != NULL)
	{
		Key->ButtonRelease();
	}
	Key->State = IDLE;
}

//State machines
void ButtonTask(button_t *Key)
{
	switch(Key->State)
	{
	case IDLE:
		ButtonIdleRoutine(Key);
		break;

	case DEBOUNCE:
		ButtonDebounceRoutine(Key);
		break;

	case PRESSED:
		ButtonPressedRoutine(Key);
		break;

	case REPEAT:
		ButtonRepeatRoutine(Key);
		break;

	case RELEASE:
		ButtonReleaseRoutine(Key);
		break;
	}
}
