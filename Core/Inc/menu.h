/*
 * menu.h
 *
 *  Created on: 21 cze 2023
 *      Author: Adrian
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include <stdint.h>
#include "stddef.h"

#define OLED_ROWS 3
#define OLED_MENU_Y_OFFSET 9
#define OLED_MENU_Y_SPACE 18
#define SCROLL_FREEZE 1000

extern const uint8_t font_8x5[];


typedef struct menu_struct menu_t;

struct menu_struct {

	const char * name;
	menu_t * next;
	menu_t * prev;
	menu_t * child;
	menu_t * parent;
	void (*menu_function)(void);
	int8_t menu_state; //0 - non on/off state | 1 - on state | -1 - off state

};

extern menu_t menu1;
extern 	menu_t sub_menu1_1;
extern 	menu_t sub_menu1_2;
extern 	menu_t sub_menu1_3;
extern 	menu_t sub_menu1_4;
extern 	menu_t sub_menu1_5;
extern 	menu_t sub_menu1_6;
extern 		menu_t sub_menu1_6_1;
extern 		menu_t sub_menu1_6_2;
extern 		menu_t sub_menu1_6_3;
extern 		menu_t sub_menu1_6_4;
extern 		menu_t sub_menu1_6_5;
extern 		menu_t sub_menu1_6_6;
extern 		menu_t sub_menu1_6_7;
extern 		menu_t sub_menu1_6_8;
extern 		menu_t sub_menu1_6_9;
extern 		menu_t sub_menu1_6_10;
extern 		menu_t sub_menu1_6_11;
extern 		menu_t sub_menu1_6_12;
extern 		menu_t sub_menu1_6_13;
extern 		menu_t sub_menu1_6_14;
extern 		menu_t sub_menu1_6_15;
extern 		menu_t sub_menu1_6_16;
extern 		menu_t sub_menu1_6_17;
extern 	menu_t sub_menu1_7;
extern menu_t menu2;
extern menu_t menu3;
extern		menu_t sub_menu3_1;
extern			menu_t sub_menu3_1_1;
extern				menu_t sub_menu3_1_1_1;
extern				menu_t sub_menu3_1_1_2;
extern				menu_t sub_menu3_1_1_3;
extern				menu_t sub_menu3_1_1_4;
extern				menu_t sub_menu3_1_1_5;
extern			menu_t sub_menu3_1_2;
extern			menu_t sub_menu3_1_3;
extern		menu_t sub_menu3_2;
extern			menu_t sub_menu3_2_1;
extern				menu_t sub_menu3_2_1_1;
extern				menu_t sub_menu3_2_1_2;
extern				menu_t sub_menu3_2_1_3;
extern				menu_t sub_menu3_2_1_4;
extern				menu_t sub_menu3_2_1_5;
extern		menu_t sub_menu3_2_2;
extern		menu_t sub_menu3_2_3;
extern		menu_t sub_menu3_3;
extern			menu_t sub_menu3_3_1;
extern			menu_t sub_menu3_3_2;
extern			menu_t sub_menu3_3_3;
extern			menu_t sub_menu3_3_4;
extern			menu_t sub_menu3_3_5;
extern		menu_t sub_menu3_4;
extern menu_t menu4;
extern 		menu_t sub_menu4_1;
extern 		menu_t sub_menu4_2;
extern 		menu_t sub_menu4_3;

extern void ShowMeasurements(void);
extern void ShowTemperature(void);
extern void ShowOut0to7(void);
extern void ShowOut8to15(void);
extern void ShowIn0to7(void);
extern void ShowIn8to15(void);
extern void ShowPWMsetMenu(void);
extern void ShowLedLightParamLogo(void);
extern void ShowLedLightParamLight(void);


void MenuNext(void);
void MenuPrev(void);
void MenuEnter(void);
void MenuBack(void);
void MenuRefresh(void);
void ScrollString(void);


#endif /* INC_MENU_H_ */
