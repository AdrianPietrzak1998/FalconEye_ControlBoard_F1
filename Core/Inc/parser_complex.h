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

typedef enum{
	ALL = 0,
	OUT,
	PWM1,
	PWM2,
	PWM3,
	PWM4

}ReceivedCommand_t;

typedef void(*CommandFun_t)(void);

const struct Command{
	ReceivedCommand_t CommandID;
	CommandFun_t Action;
	uint8_t CommandArgQ;
};

extern void all(void);
extern void out(void);
extern void pwm1(void);
extern void pwm2(void);
extern void pwm3(void);
extern void pwm4(void);


void Parser_TakeLine(RingBuffer_t *Buff, uint8_t *Destination);
void Parser_parse(uint8_t * DataToParse);

extern void UsbBuffWrite(char * Message);

#endif /* INC_PARSER_COMPLEX_H_ */
