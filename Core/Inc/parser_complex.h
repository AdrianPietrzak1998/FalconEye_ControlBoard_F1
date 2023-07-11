/*
 * parser_simple.h
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#ifndef INC_PARSER_COMPLEX_H_
#define INC_PARSER_COMPLEX_H_

#define ENDLINE '^'

extern uint8_t FanSpeed;

/*
 * OUT_REG (ODR_VALUE)
 * OTU_PIN_SET (PIN 0 - 15, STATE 0|1)
 * OUT_PIN_TOGGLE (PIN 0 - 15)
 * PWM_ALL (PWM1, PWM2 0 - 1000, ...)
 * PWM_CHANNEL_SET (PWM_CHANNEL 1 - 4, PWM 0 - 1000)
 * DISPLAY_CONTRAST (CONTRAST 0 - 255)
 * LIGHT_PARAMETER (MODE -1 - 3, MAXIMUM PWM VALUE 1 - 1000, DIMMER SPEED 1 - 65535)
 * LOGO_PARAMETER (MODE -1 - 3, MAXIMUM PWM VALUE 1 - 1000, DIMMER SPEED 1 - 65535)
 */

typedef enum{
	OUT_REG = 0,
	OUT_PIN_SET,
	OUT_PIN_TOGGLE,
	PWM_ALL,
	PWM_CHANNEL_SET,
	DISPLAY_CONTRAST,
	LIGHT_PARAM,
	LOGO_PARAM

}ReceivedCommand_t;

typedef void(*CommandFun_t)();

const struct Command{
	ReceivedCommand_t CommandID;
	CommandFun_t Action;
	uint8_t CommandArgQ;
};
//
// extern declaration received function
//
extern void OutputSet(uint16_t ODRvalue);
extern void OutputPinStateSet(uint8_t Pin, uint8_t State);
extern void OutputPinToggle(uint8_t Pin);
extern void PwmSet(uint16_t Pwm1, uint16_t Pwm2, uint16_t Pwm3, uint16_t Pwm4);
extern void PwmChannelSet(uint8_t Channel, uint16_t Value);
extern void DisplayContrast(uint8_t Contrast);
extern void LightLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);
extern void LogoLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);
//
//
//


void Parser_TakeLine(RingBuffer_t *Buff, uint8_t *Destination);
void Parser_parse(uint8_t * DataToParse);

extern void UsbBuffWrite(char * Message);

#endif /* INC_PARSER_COMPLEX_H_ */
