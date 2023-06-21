/*
 * button.h
 *
 *  Created on: May 7, 2022
 *      Author: Adrian
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

// States of state machine

typedef enum
{
	IDLE = 0,
	DEBOUNCE,
	PRESSED,
	REPEAT,
	RELEASE

} BUTTON_STATE;

// Struct for buttons

typedef struct
{
	BUTTON_STATE  State;

	GPIO_TypeDef *GpioPort;
	uint16_t      GpioPin;

	uint32_t      LastTick;
	uint32_t      TimerDebounce;
	uint32_t	  TimerLongPressed;
	uint32_t	  TimerRepeat;

	void(*ButtonPressed)(void);
	void(*ButtonLongPressed)(void);
	void(*ButtonRepeat)(void);
	void(*ButtonRelease)(void);

} button_t;

// Public function
void ButtonInitKey(button_t * Key, GPIO_TypeDef *GpioPort, uint16_t GpioPin, uint32_t TimerDebounce, uint32_t TimerLongPressed,
					uint32_t TimerRepeat); // Initialization for state machine
void ButtonTask(button_t *Key); //Task for working state machine

//Registers functions need Callback as a pointer, type name of function without '()'
void ButtonRegisterPressCallback(button_t *Key, void *Callback); //For first press key
void ButtonRegisterLongPressedCallback(button_t *Key, void *Callback); //If key was long pressed
void ButtonRegisterRepeatCallback(button_t *Key, void *Callback); //While key is long pressed
void ButtonRegisterReleaseCallback(button_t *Key, void *Callback); //If key was released

void ButtonSetDebounceTime(button_t * Key, uint32_t Miliseconds);
void ButtonSetLongPressedTime(button_t *Key, uint32_t Miliseconds);
void ButtonSetRepeatTime(button_t *Key, uint32_t Miliseconds);

#endif /* INC_BUTTON_H_ */
