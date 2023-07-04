/*
 * menu.c
 *
 *  Created on: 21 cze 2023
 *      Author: Adrian
 */


#include "menu.h"
#include "main.h"
#include "stddef.h"
#include "ssd1306.h"
#include "GFX_BW.h"
#include "string.h"
#include "menu_out_set.h"

menu_t *CurrentPointer = &menu1; //Actual menu level
uint8_t MenuIndex; //Actual menu index on screen
uint8_t OledRowPos; //Actual menu row select
uint8_t OledRowPosLevel[5]; //Save past menu row. Arr size == max menu level

const char BackStr[] = "Back";

int16_t TempStrLengthVar;
char *TempStr;
uint8_t TempId;
uint8_t ScrollEn;
uint8_t ScrollFirst;


static void StateIndicator(menu_t *menu, uint8_t pos);
static uint8_t MenuGetIndex(menu_t *menu);
static uint8_t MenuGetLevel(menu_t *menu);
static void HeaderDraw(char *header);

menu_t menu1 = { "I/O", &menu2, &menu6, &sub_menu1_1, NULL, NULL, 0 };
	menu_t sub_menu1_1 = { "OUT 0-7", &sub_menu1_2, &sub_menu1_7, NULL, &menu1, ShowOut0to7 };
	menu_t sub_menu1_2 = { "OUT 8-15", &sub_menu1_3, &sub_menu1_1, NULL, &menu1, ShowOut8to15 };
	menu_t sub_menu1_3 = { "PWM", &sub_menu1_4, &sub_menu1_2, NULL, &menu1, ShowPWMsetMenu };
	menu_t sub_menu1_4 = { "IN 0-7", &sub_menu1_5, &sub_menu1_3, NULL, &menu1, ShowIn0to7 };
	menu_t sub_menu1_5 = { "IN 8-15", &sub_menu1_6, &sub_menu1_4, NULL, &menu1, ShowIn8to15 };
	menu_t sub_menu1_6 = { "OUT settings", &sub_menu1_7, &sub_menu1_5, &sub_menu1_6_1, &menu1, NULL };
		menu_t sub_menu1_6_1  = { "OUT 0",  &sub_menu1_6_2,  &sub_menu1_6_17, NULL, &sub_menu1_6, SetOut0 };
		menu_t sub_menu1_6_2  = { "OUT 1",  &sub_menu1_6_3,  &sub_menu1_6_1,  NULL, &sub_menu1_6, SetOut1 };
		menu_t sub_menu1_6_3  = { "OUT 2",  &sub_menu1_6_4,  &sub_menu1_6_2,  NULL, &sub_menu1_6, SetOut2 };
		menu_t sub_menu1_6_4  = { "OUT 3",  &sub_menu1_6_5,  &sub_menu1_6_3,  NULL, &sub_menu1_6, SetOut3 };
		menu_t sub_menu1_6_5  = { "OUT 4",  &sub_menu1_6_6,  &sub_menu1_6_4,  NULL, &sub_menu1_6, SetOut4 };
		menu_t sub_menu1_6_6  = { "OUT 5",  &sub_menu1_6_7,  &sub_menu1_6_5,  NULL, &sub_menu1_6, SetOut5 };
		menu_t sub_menu1_6_7  = { "OUT 6",  &sub_menu1_6_8,  &sub_menu1_6_6,  NULL, &sub_menu1_6, SetOut6 };
		menu_t sub_menu1_6_8  = { "OUT 7",  &sub_menu1_6_9,  &sub_menu1_6_7,  NULL, &sub_menu1_6, SetOut7 };
		menu_t sub_menu1_6_9  = { "OUT 8",  &sub_menu1_6_10, &sub_menu1_6_8,  NULL, &sub_menu1_6, SetOut8 };
		menu_t sub_menu1_6_10 = { "OUT 9",  &sub_menu1_6_11, &sub_menu1_6_9,  NULL, &sub_menu1_6, SetOut9 };
		menu_t sub_menu1_6_11 = { "OUT 10", &sub_menu1_6_12, &sub_menu1_6_10, NULL, &sub_menu1_6, SetOut10 };
		menu_t sub_menu1_6_12 = { "OUT 11", &sub_menu1_6_13, &sub_menu1_6_11, NULL, &sub_menu1_6, SetOut11 };
		menu_t sub_menu1_6_13 = { "OUT 12", &sub_menu1_6_14, &sub_menu1_6_12, NULL, &sub_menu1_6, SetOut12 };
		menu_t sub_menu1_6_14 = { "OUT 13", &sub_menu1_6_15, &sub_menu1_6_13, NULL, &sub_menu1_6, SetOut13 };
		menu_t sub_menu1_6_15 = { "OUT 14", &sub_menu1_6_16, &sub_menu1_6_14, NULL, &sub_menu1_6, SetOut14 };
		menu_t sub_menu1_6_16 = { "OUT 15", &sub_menu1_6_17, &sub_menu1_6_15, NULL, &sub_menu1_6, SetOut15 };
		menu_t sub_menu1_6_17 = { BackStr, NULL, &sub_menu1_6_16, NULL, &sub_menu1_6, MenuBack };
	menu_t sub_menu1_7 = { BackStr, NULL, &sub_menu1_6, NULL, &menu1, MenuBack };
menu_t menu2 = { "ELEMENT 2", &menu3, &menu1, &sub_menu2_1, NULL, NULL, 0 };
	menu_t sub_menu2_1 = { "ELEMENT 2_1", &sub_menu2_2, &sub_menu2_4, NULL, &menu2, NULL, 0 };
	menu_t sub_menu2_2 = { "ELEMENT 2_2", &sub_menu2_3, &sub_menu2_1, &sub_menu2_2_1, &menu2, NULL, 0 };
		menu_t sub_menu2_2_1 = { "ELEMENT 2_2_1", &sub_menu2_2_2, &sub_menu2_2_5, NULL, &sub_menu2_2, NULL, 1 };
		menu_t sub_menu2_2_2 = { "ELEMENT 2_2_2", &sub_menu2_2_3, &sub_menu2_2_1, NULL, &sub_menu2_2, NULL, -1 };
		menu_t sub_menu2_2_3 = { "ELEMENT 2_2_3", &sub_menu2_2_4, &sub_menu2_2_2, NULL, &sub_menu2_2, NULL, 0 };
		menu_t sub_menu2_2_4 = { "test", &sub_menu2_2_5, &sub_menu2_2_3, NULL, &sub_menu2_2, NULL, -1 };
		menu_t sub_menu2_2_5 = { BackStr, NULL, &sub_menu2_2_4, NULL, &sub_menu2_2, MenuBack };
	menu_t sub_menu2_3 = { "ELEMENT 2_3", &sub_menu2_4, &sub_menu2_2, NULL, &menu2, NULL };
	menu_t sub_menu2_4 = { BackStr, NULL, &sub_menu2_3, NULL, &menu2, MenuBack };
menu_t menu3 = { "ELEMENT 3", &menu4, &menu2, NULL, NULL, NULL };
menu_t menu4 = { "ELEMENT 4", &menu5, &menu3, NULL, NULL, NULL };
menu_t menu5 = { "ELEMENT 5", &menu6, &menu4, NULL, NULL, NULL };
menu_t menu6 = { "Monitor", NULL, &menu5, &sub_menu6_1, NULL, NULL };
	menu_t sub_menu6_1 = { "Measurements", &sub_menu6_2, &sub_menu6_3, NULL, &menu6, ShowMeasurements };
	menu_t sub_menu6_2 = { "Temperature", &sub_menu6_3, &sub_menu6_1, NULL, &menu6, ShowTemperature };
	menu_t sub_menu6_3 = { BackStr, NULL, &sub_menu6_2, NULL, &menu6, MenuBack };


static void HeaderDraw(char *header)

{
	  GFX_SetFont(font_8x5);
	  GFX_SetFontSize(1);
	  size_t headerLen = strlen(header);
	  char buff[20];
	  if(headerLen % 2)
	  {
		  for(uint8_t i = 0; i<19; i++)
		  {
			  if(i<9-(headerLen/2))
			  {
				  buff[i] = '-';
			  }
			  else if(i>=9-(headerLen/2) && i<9+(headerLen/2) + 1)
			  {
				  buff[i] = header[i-9+(headerLen/2)];
			  }
			  else
			  {
				  buff[i] = '-';
			  }
		  }
		  buff[19] = 0;
		  GFX_DrawString(9, 0, buff, WHITE, 0);
	  }
	  else
	  {
		  for(uint8_t i = 0; i<18; i++)
		  {
			  if(i<9-(headerLen/2))
			  {
				  buff[i] = '-';
			  }
			  else if(i>=9-(headerLen/2) && i<9+(headerLen/2))
			  {
				  buff[i] = header[i-9+(headerLen/2)];
			  }
			  else
			  {
				  buff[i] = '-';
			  }
		  }
		  buff[18] = 0;
		  GFX_DrawString(11, 0, buff, WHITE, 0);
	  }
}

void MenuNext(void)
{
	if(CurrentPointer->next)
	{
		CurrentPointer = CurrentPointer->next;
		MenuIndex++;
		if(++OledRowPos > OLED_ROWS - 1)
		{
			OledRowPos = OLED_ROWS - 1;
		}
	}
	else
	{
		MenuIndex = 0;
		OledRowPos = 0;
		if(CurrentPointer->parent)
		{
			CurrentPointer = (CurrentPointer -> parent) -> child;
		}
		else
		{
			CurrentPointer = &menu1;
		}
	}
	MenuRefresh();
}

void MenuPrev(void)
{
	CurrentPointer = CurrentPointer -> prev;

	if(MenuIndex)
	{
		MenuIndex--;
		if(OledRowPos > 0)
		{
			OledRowPos--;
		}
	}
	else
	{
		MenuIndex = MenuGetIndex(CurrentPointer);
		{
			if(MenuIndex >= OLED_ROWS)
			{
				OledRowPos = OLED_ROWS - 1;
			}
			else
			{
				OledRowPos = MenuIndex;
			}
		}
	}
	MenuRefresh();
}

void MenuEnter(void)
{
	uint8_t Back = 0;

	if(CurrentPointer->menu_function != NULL && CurrentPointer->menu_function !=MenuBack)
	{
		CurrentPointer -> menu_function();
	}
	else if(CurrentPointer->menu_function != NULL && CurrentPointer->menu_function == MenuBack)
	{
		Back = 1;
	}

	if(CurrentPointer->child != NULL)
	{

			OledRowPosLevel[MenuGetLevel(CurrentPointer)] = OledRowPos;

		MenuIndex = 0;
		OledRowPos = 0;
		CurrentPointer = CurrentPointer -> child;
	}

	if(Back == 1)
	{
		MenuBack();
	}

	MenuRefresh();
}

void MenuBack(void)
{
	if(CurrentPointer->parent != NULL)
	{
		CurrentPointer = CurrentPointer -> parent;
		OledRowPos = OledRowPosLevel[MenuGetLevel(CurrentPointer)];
	}

//	CurrentPointer = CurrentPointer -> parent;
	MenuIndex = MenuGetIndex(CurrentPointer);

	MenuRefresh();
}

static uint8_t MenuGetIndex(menu_t *menu)
{
	menu_t *temp;
	uint8_t i = 0;

	if(menu->parent)
	{
		temp = (menu->parent) -> child;
	}
	else
	{
		temp = &menu1;
	}

	while(temp != menu)
	{
		temp = temp -> next;
		i++;
	}

	return i;
}

uint8_t MenuGetLevel(menu_t *menu)
{
	menu_t *temp = menu;
	uint8_t i = 0;

	if(menu->parent == NULL)
	{
		return 0;
	}

	while(temp->parent != NULL)
	{
		temp = temp -> parent;
		i++;
	}
	return i;
}

void MenuRefresh(void)
{
	menu_t *temp;
	uint8_t i;

	if(CurrentPointer->parent == &sub_menu1_6)
	{
		MenuIndicatorRefresh();
	}

	SSD1306_Clear(BLACK);

	if(CurrentPointer->parent)
	{
		temp = (CurrentPointer->parent) -> child;
		HeaderDraw((CurrentPointer->parent) -> name);
	}
	else
	{
		temp = &menu1;
		HeaderDraw("Menu");
	}

	for(i=0; i!=MenuIndex - OledRowPos; i++)
	{
		if(temp -> next == NULL) break; // Hard fault protection

		temp = temp -> next;
	}

	GFX_SetFont(font_8x5);
	GFX_SetFontSize(2);
	for(i=0; i<OLED_ROWS; i++)
	{
		if(temp == CurrentPointer)
		{
//			GFX_DrawFillRectangle(5, (i*20) + OLED_MENU_Y_OFFSET, 123, 16, WHITE);
//			GFX_DrawFillCircle(10, (i*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
//			GFX_DrawString(20, (i*20) + OLED_MENU_Y_OFFSET + 1, temp->name, BLACK, 1);

			TempStr = temp->name;
			uint8_t TempStrLength = strlen(TempStr);
			TempStrLengthVar = -(TempStrLength * 5);
			TempId = i;

			if(TempStrLength >= 10)
			{
				ScrollEn = 1;
				ScrollFirst =1;
				ScrollString();
			}
			else
			{
				ScrollEn = 0;
				GFX_DrawRectangle(17, ((i*OLED_MENU_Y_SPACE)-1) + OLED_MENU_Y_OFFSET, 111, 18, WHITE);
//				GFX_DrawFillCircle(10, (i*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
//				GFX_DrawChar(5, (i*20) + OLED_MENU_Y_OFFSET + 1, '>', BLACK, 1);
				StateIndicator(temp, i);
				GFX_DrawString(20, (i*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, temp->name, WHITE, 1);
			}

		}
		else
		{
//			GFX_DrawChar(5, (i*20) + OLED_MENU_Y_OFFSET + 1, '>', WHITE, 0);
			StateIndicator(temp, i);
			GFX_DrawString(20, (i*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET, temp->name, WHITE, 0);
		}

		temp = temp -> next;
		if(!temp) break;
	}
}

void ScrollString(void)
{
	  static int16_t ScrollVar = 20;
	  static int16_t ScrollVar2 = 123;

	  static uint32_t OldTick = 0;

	  if(ScrollFirst)
	  {
		  ScrollFirst = 0;
		  ScrollVar = 20;
		  ScrollVar2 = 123;
		  OldTick = HAL_GetTick();
	  }

	  if(ScrollEn)
	  {

	  int16_t var = TempStrLengthVar;

	  	GFX_DrawFillRectangle(0, ((TempId*OLED_MENU_Y_SPACE)-1) + OLED_MENU_Y_OFFSET, 128, 18, BLACK);
		if(ScrollVar>var-75)
		{
		GFX_DrawString(ScrollVar, (TempId*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, TempStr, WHITE, 1);
		}

		GFX_DrawFillRectangle(0, ((TempId*OLED_MENU_Y_SPACE)-1) + OLED_MENU_Y_OFFSET, 17, 18, BLACK);
//		GFX_DrawFillCircle(10, (TempId*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
//		GFX_DrawChar(5, (TempId*20) + OLED_MENU_Y_OFFSET + 1, '>', BLACK, 1);
		StateIndicator(CurrentPointer, TempId);
		GFX_DrawRectangle(17, ((TempId*OLED_MENU_Y_SPACE)-1) + OLED_MENU_Y_OFFSET, 111, 18, WHITE);

		if(HAL_GetTick() - OldTick >= SCROLL_FREEZE) ScrollVar--;

		  if(ScrollVar < var)
		  {
			  GFX_DrawString(ScrollVar2, (TempId*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, TempStr, WHITE, 1);
			  ScrollVar2--;
			  if(ScrollVar2 == 20)
			  {
				  ScrollVar = 20;
				  ScrollVar2 = 123;
			  }
		  }

	  }

}

static void StateIndicator(menu_t *menu, uint8_t pos)
{
	if(menu == CurrentPointer)
	{
		if(menu->menu_state == 0 && menu->name != BackStr)
		{
			GFX_DrawChar(5, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, '>', WHITE, 0);
		}
		else if(menu->menu_state == 0 && menu->name == BackStr)
		{
			GFX_DrawChar(5, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, '<', WHITE, 0);
		}
		else if(menu->menu_state == 1)
		{
			GFX_DrawFillCircle(10, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
		else if(menu->menu_state == -1)
		{
			GFX_DrawCircle(10, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
	}
	else
	{
		if(menu->menu_state == 0 && menu->name != BackStr)
		{
			GFX_DrawChar(5, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, '>', WHITE, 0);
		}
		else if(menu->menu_state == 0 && menu->name == BackStr)
		{
			GFX_DrawChar(5, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 1, '<', WHITE, 0);
		}
		else if(menu->menu_state == 1)
		{
			GFX_DrawFillCircle(10, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
		else if(menu->menu_state == -1)
		{
			GFX_DrawCircle(10, (pos*OLED_MENU_Y_SPACE) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
	}
}
























