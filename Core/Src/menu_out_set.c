/*
 * menu_out_set.c
 *
 *  Created on: Jul 4, 2023
 *      Author: Adrian
 */

#include "main.h"
#include "menu_out_set.h"
#include "menu.h"

extern LedLightParameter_t Logo;
extern LedLightParameter_t Light;

extern TIM_HandleTypeDef htim4;

extern enum PwmFreqency;

void MenuIndicatorRefresh(void)
{
	sub_menu1_6_1.menu_state =  ((GPIOE -> ODR >> 0)  & 1)? 1 : -1;
	sub_menu1_6_2.menu_state =  ((GPIOE -> ODR >> 1)  & 1)? 1 : -1;
	sub_menu1_6_3.menu_state =  ((GPIOE -> ODR >> 2)  & 1)? 1 : -1;
	sub_menu1_6_4.menu_state =  ((GPIOE -> ODR >> 3)  & 1)? 1 : -1;
	sub_menu1_6_5.menu_state =  ((GPIOE -> ODR >> 4)  & 1)? 1 : -1;
	sub_menu1_6_6.menu_state =  ((GPIOE -> ODR >> 5)  & 1)? 1 : -1;
	sub_menu1_6_7.menu_state =  ((GPIOE -> ODR >> 6)  & 1)? 1 : -1;
	sub_menu1_6_8.menu_state =  ((GPIOE -> ODR >> 7)  & 1)? 1 : -1;
	sub_menu1_6_9.menu_state =  ((GPIOE -> ODR >> 8)  & 1)? 1 : -1;
	sub_menu1_6_10.menu_state = ((GPIOE -> ODR >> 9)  & 1)? 1 : -1;
	sub_menu1_6_11.menu_state = ((GPIOE -> ODR >> 10) & 1)? 1 : -1;
	sub_menu1_6_12.menu_state = ((GPIOE -> ODR >> 11) & 1)? 1 : -1;
	sub_menu1_6_13.menu_state = ((GPIOE -> ODR >> 12) & 1)? 1 : -1;
	sub_menu1_6_14.menu_state = ((GPIOE -> ODR >> 13) & 1)? 1 : -1;
	sub_menu1_6_15.menu_state = ((GPIOE -> ODR >> 14) & 1)? 1 : -1;
	sub_menu1_6_16.menu_state = ((GPIOE -> ODR >> 15) & 1)? 1 : -1;

	sub_menu3_1_1_1.menu_state = (Logo.Mode == 0)? 1 : -1;
	sub_menu3_1_1_2.menu_state = (Logo.Mode == 1)? 1 : -1;
	sub_menu3_1_1_3.menu_state = (Logo.Mode == 2)? 1 : -1;
	sub_menu3_1_1_4.menu_state = (Logo.Mode == 3)? 1 : -1;

	sub_menu3_2_1_1.menu_state = (Light.Mode == 0)? 1 : -1;
	sub_menu3_2_1_2.menu_state = (Light.Mode == 1)? 1 : -1;
	sub_menu3_2_1_3.menu_state = (Light.Mode == 2)? 1 : -1;
	sub_menu3_2_1_4.menu_state = (Light.Mode == 3)? 1 : -1;

	sub_menu3_3_1.menu_state = (htim4.Init.Prescaler ==  HZ60)? 1 : -1;
	sub_menu3_3_2.menu_state = (htim4.Init.Prescaler == HZ100)? 1 : -1;
	sub_menu3_3_3.menu_state = (htim4.Init.Prescaler == HZ200)? 1 : -1;
	sub_menu3_3_4.menu_state = (htim4.Init.Prescaler == HZ300)? 1 : -1;
}

void SetOut0(void)
{
	OutputPinToggle(0);
}

void SetOut1(void)
{
	OutputPinToggle(1);
}

void SetOut2(void)
{
	OutputPinToggle(2);
}

void SetOut3(void)
{
	OutputPinToggle(3);
}

void SetOut4(void)
{
	OutputPinToggle(4);
}

void SetOut5(void)
{
	OutputPinToggle(5);
}

void SetOut6(void)
{
	OutputPinToggle(6);
}

void SetOut7(void)
{
	OutputPinToggle(7);
}

void SetOut8(void)
{
	OutputPinToggle(8);
}

void SetOut9(void)
{
	OutputPinToggle(9);
}

void SetOut10(void)
{
	OutputPinToggle(10);
}

void SetOut11(void)
{
	OutputPinToggle(11);
}

void SetOut12(void)
{
	OutputPinToggle(12);
}

void SetOut13(void)
{
	OutputPinToggle(13);
}

void SetOut14(void)
{
	OutputPinToggle(14);
}

void SetOut15(void)
{
	OutputPinToggle(15);
}

void SetModeLogo0(void)
{
	LogoLedSetParameter(0, 0, 0);
}

void SetModeLogo1(void)
{
	LogoLedSetParameter(1, 0, 0);
}

void SetModeLogo2(void)
{
	LogoLedSetParameter(2, 0, 0);
}

void SetModeLogo3(void)
{
	LogoLedSetParameter(3, 0, 0);
}

void SetModeLight0(void)
{
	LightLedSetParameter(0, 0, 0);
}
void SetModeLight1(void)
{
	LightLedSetParameter(1, 0, 0);
}
void SetModeLight2(void)
{
	LightLedSetParameter(2, 0, 0);
}
void SetModeLight3(void)
{
	LightLedSetParameter(3, 0, 0);
}

void SetPwmFreq60(void)
{
	PwmFreqSet(HZ60);
}

void SetPwmFreq100(void)
{
	PwmFreqSet(HZ100);
}

void SetPwmFreq200(void)
{
	PwmFreqSet(HZ200);
}

void SetPwmFreq300(void)
{
	PwmFreqSet(HZ300);
}

