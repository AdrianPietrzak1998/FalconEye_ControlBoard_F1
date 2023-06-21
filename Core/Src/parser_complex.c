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



uint16_t ReceivedCommandArgument[16];

struct Command CommandMapper[] = {
		{ALL, all, 5},
		{OUT, all, 1},
		{PWM1, all, 1},
		{PWM2, all, 1},
		{PWM3, all, 1},
		{PWM4, all, 1},
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

		CommandMapper[CommandID].Action();
	}
}
