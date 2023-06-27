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
extern 		menu_t sub_menu1_1_1;
extern 	menu_t sub_menu1_2;
extern menu_t menu2;
extern 	menu_t sub_menu2_1;
extern 	menu_t sub_menu2_2;
extern 		menu_t sub_menu2_2_1;
extern 		menu_t sub_menu2_2_2;
extern 		menu_t sub_menu2_2_3;
extern 		menu_t sub_menu2_2_4;
extern 		menu_t sub_menu2_2_5;
extern 	menu_t sub_menu2_3;
extern 	menu_t sub_menu2_4;
extern menu_t menu3;
extern menu_t menu4;
extern menu_t menu5;
extern menu_t menu6;



void HeaderDraw(char *header);
void MenuNext(void);
void MenuPrev(void);
void MenuEnter(void);
void MenuBack(void);
uint8_t MenuGetIndex(menu_t *menu);
uint8_t MenuGetLevel(menu_t *menu);
void MenuRefresh(void);
void ScrollString(void);


#endif /* INC_MENU_H_ */
