/*
 * menu_out_set.h
 *
 *  Created on: Jul 4, 2023
 *      Author: Adrian
 */

#ifndef INC_MENU_OUT_SET_H_
#define INC_MENU_OUT_SET_H_

#include <stdint.h>
#include "stddef.h"

extern void OutputPinToggle(uint8_t Pin);

extern void LogoLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);
extern void LightLedSetParameter(uint8_t Mode, uint16_t PwmValue, uint16_t DimmerSpeed);

void SetOut0(void);
void SetOut1(void);
void SetOut2(void);
void SetOut3(void);
void SetOut4(void);
void SetOut5(void);
void SetOut6(void);
void SetOut7(void);
void SetOut8(void);
void SetOut9(void);
void SetOut10(void);
void SetOut11(void);
void SetOut12(void);
void SetOut13(void);
void SetOut14(void);
void SetOut15(void);

void SetModeLogo0(void);
void SetModeLogo1(void);
void SetModeLogo2(void);
void SetModeLogo3(void);
void SetModeLight0(void);
void SetModeLight1(void);
void SetModeLight2(void);
void SetModeLight3(void);

void SetPwmFreq60(void);
void SetPwmFreq100(void);
void SetPwmFreq200(void);
void SetPwmFreq300(void);

void MenuIndicatorRefresh(void);

#endif /* INC_MENU_OUT_SET_H_ */
