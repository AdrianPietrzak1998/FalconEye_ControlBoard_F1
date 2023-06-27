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

menu_t menu1 = { "ELEMENT 1", &menu2, &menu6, &sub_menu1_1, NULL, NULL, 0 };
	menu_t sub_menu1_1 = { "ELEMENT 1_1", &sub_menu1_2, &sub_menu1_2, &sub_menu1_1_1, &menu1, NULL, 0 };
		menu_t sub_menu1_1_1 = { BackStr, NULL, NULL, NULL, &sub_menu1_1, MenuBack };
	menu_t sub_menu1_2 = { BackStr, NULL, &sub_menu1_1, NULL, &menu1, MenuBack };
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
menu_t menu6 = { "ELEMENT 6", NULL, &menu5, NULL, NULL, NULL };


void HeaderDraw(char *header)

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

uint8_t MenuGetIndex(menu_t *menu)
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
				GFX_DrawFillRectangle(5, (i*20) + OLED_MENU_Y_OFFSET, 123, 16, WHITE);
//				GFX_DrawFillCircle(10, (i*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
//				GFX_DrawChar(5, (i*20) + OLED_MENU_Y_OFFSET + 1, '>', BLACK, 1);
				StateIndicator(temp, i);
				GFX_DrawString(20, (i*20) + OLED_MENU_Y_OFFSET + 1, temp->name, BLACK, 1);
			}

		}
		else
		{
//			GFX_DrawChar(5, (i*20) + OLED_MENU_Y_OFFSET + 1, '>', WHITE, 0);
			StateIndicator(temp, i);
			GFX_DrawString(20, (i*20) + OLED_MENU_Y_OFFSET, temp->name, WHITE, 0);
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

		GFX_DrawFillRectangle(5, (TempId*20) + OLED_MENU_Y_OFFSET, 123, 16, WHITE);
		if(ScrollVar>var-75)
		{
		GFX_DrawString(ScrollVar, (TempId*20) + OLED_MENU_Y_OFFSET + 1, TempStr, BLACK, 1);
		}

		GFX_DrawFillRectangle(5, (TempId*20) + OLED_MENU_Y_OFFSET, 15, 16, WHITE);
//		GFX_DrawFillCircle(10, (TempId*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
//		GFX_DrawChar(5, (TempId*20) + OLED_MENU_Y_OFFSET + 1, '>', BLACK, 1);
		StateIndicator(CurrentPointer, TempId);

		if(HAL_GetTick() - OldTick >= SCROLL_FREEZE) ScrollVar--;

		  if(ScrollVar < var)
		  {
			  GFX_DrawString(ScrollVar2, (TempId*20) + OLED_MENU_Y_OFFSET + 1, TempStr, BLACK, 1);
			  ScrollVar2--;
			  if(ScrollVar2 == 20)
			  {
				  ScrollVar = 20;
				  ScrollVar2 = 123;
			  }
		  }

	  }

}

void StateIndicator(menu_t *menu, uint8_t pos)
{
	if(menu == CurrentPointer)
	{
		if(menu->menu_state == 0 && menu->name != BackStr)
		{
			GFX_DrawChar(5, (pos*20) + OLED_MENU_Y_OFFSET + 1, '>', BLACK, 1);
		}
		else if(menu->menu_state == 0 && menu->name == BackStr)
		{
			GFX_DrawChar(5, (pos*20) + OLED_MENU_Y_OFFSET + 1, '<', BLACK, 1);
		}
		else if(menu->menu_state == 1)
		{
			GFX_DrawFillCircle(10, (pos*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
		}
		else if(menu->menu_state == -1)
		{
			GFX_DrawCircle(10, (pos*20) + OLED_MENU_Y_OFFSET + 8, 4, BLACK);
		}
	}
	else
	{
		if(menu->menu_state == 0 && menu->name != BackStr)
		{
			GFX_DrawChar(5, (pos*20) + OLED_MENU_Y_OFFSET + 1, '>', WHITE, 0);
		}
		else if(menu->menu_state == 0 && menu->name == BackStr)
		{
			GFX_DrawChar(5, (pos*20) + OLED_MENU_Y_OFFSET + 1, '<', WHITE, 0);
		}
		else if(menu->menu_state == 1)
		{
			GFX_DrawFillCircle(10, (pos*20) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
		else if(menu->menu_state == -1)
		{
			GFX_DrawCircle(10, (pos*20) + OLED_MENU_Y_OFFSET + 8, 4, WHITE);
		}
	}
}
























