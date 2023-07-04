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

void MenuIndicatorRefresh(void);

#endif /* INC_MENU_OUT_SET_H_ */
