/*
 * parser_complex.c
 *
 *  Created on: 4 cze 2022
 *      Author: Adrian
 */

#include "main.h"
#include "ring_buffer.h"
#include "utils.h"
#include "parser_complex.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#define ReceivedArgumentSize 16


int32_t ReceivedCommandArgument[ReceivedArgumentSize];

struct Command CommandMapper[] = {
		{OUT_REG, OutputSet, 1},
		{OUT_PIN_SET, OutputPinStateSet, 2},
		{OUT_PIN_TOGGLE, OutputPinToggle, 1},
		{PWM_ALL, PwmSet, 4},
		{PWM_CHANNEL_SET, PwmChannelSet, 2},
		{DISPLAY_CONTRAST, DisplayContrast, 1},
		{LIGHT_PARAM, LightLedSetParameter, 3},
		{LOGO_PARAM, LogoLedSetParameter, 3}
};

void Parser_TakeLine(RingBuffer_t *Buff, uint8_t *Destination)
{
	  uint8_t i = 0;
	  uint8_t tmp = 0;
	do
	{
		 Ring_Buffer_Read(Buff, &tmp);
		 if(tmp == ENDLINE)
			{
			 Destination[i] = 0;
			}
		else
			{
			Destination[i] = tmp;
			}

			i++;

	} while(tmp != ENDLINE);
}




void Parser_parse(uint8_t * DataToParse)
{
	char * ParsePointer = strtok((char*)DataToParse, "/");
	uint8_t CommandID = atoi(ParsePointer);

	if(CommandMapper[CommandID].Action != NULL)
	{
		for(uint8_t i = 0; i<CommandMapper[CommandID].CommandArgQ; i++)
		{
			char * ParsePointer = strtok(NULL, "/");
			ReceivedCommandArgument[i] = atoi(ParsePointer);
		}

 		CommandMapper[CommandID].Action(ReceivedCommandArgument[0], ReceivedCommandArgument[1], ReceivedCommandArgument[2], ReceivedCommandArgument[3]);

 		//Earising argument buffer
 		for(uint8_t i=0; i<ReceivedArgumentSize; i++)
 		{
 			ReceivedCommandArgument[i] = 0;
 		}
 		UsbBuffWrite("OK");
	}
}
