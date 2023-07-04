/*
 * ssd1306.c
 *
 *  Created on: 27 lut 2023
 *      Author: Adrian
 */

#include "main.h"
#include "ssd1306.h"
#include "string.h"
#include "dma.h"

static I2C_HandleTypeDef *oled_i2c;
#ifdef SSD1306_USE_DMA_DOUBLE_BUFFERING
static DMA_HandleTypeDef *oled_buff_dma;
#endif

static uint8_t buffer[SSD1306_BUFFER_SIZE];
static uint8_t buffer_disp[SSD1306_BUFFER_SIZE];

void SSD1306_Command(uint8_t Command)
{
	HAL_I2C_Mem_Write(oled_i2c, (SSD1306_ADRESS<<1), 0x00, 1, &Command, 1, SSD1306_TIMEOUT);
}

static void SSD1306_Data(uint8_t *Data, uint16_t Size)
{
#ifdef SSD1306_USE_DMA

	if(oled_i2c -> hdmatx -> State == HAL_DMA_STATE_READY)
	{
		HAL_I2C_Mem_Write_DMA(oled_i2c, (SSD1306_ADRESS<<1), 0x40, 1, Data, Size);
	}
#else
	HAL_I2C_Mem_Write(oled_i2c, (SSD1306_ADRESS<<1), 0x40, 1, Data, Size, SSD1306_TIMEOUT);
#endif
}

void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t Color)
{
	if((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT))
	  return;

    switch(Color)
    {
    case SSD1306_WHITE:
      buffer[x + (y / 8) * SSD1306_LCDWIDTH] |= (1 << (y & 7));
      break;
    case SSD1306_BLACK:
      buffer[x + (y / 8) * SSD1306_LCDWIDTH] &= ~(1 << (y & 7));
      break;
    case SSD1306_INVERSE:
      buffer[x + (y / 8) * SSD1306_LCDWIDTH] ^= (1 << (y & 7));
      break;
    }
}


void SSD1306_Clear(uint8_t Color)
{
	switch(Color)
	{
	case WHITE:
		memset(buffer, 0xff, SSD1306_BUFFER_SIZE);
		break;
	case BLACK:
		memset(buffer, 0x00, SSD1306_BUFFER_SIZE);
		break;
	}
}

void SSD1306_Display(void)
{
	SSD1306_Command(SSD1306_PAGEADDR);
	SSD1306_Command(0);                      // Page start address
	SSD1306_Command(0xFF);                   // Page end (not really, but works here)
	SSD1306_Command(SSD1306_COLUMNADDR);
	SSD1306_Command(0); // Column start address
	SSD1306_Command(SSD1306_LCDWIDTH - 1); // Column end address

#ifdef SSD1306_USE_DMA_DOUBLE_BUFFERING
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_channel1, (uint32_t*)buffer, (uint32_t*)buffer_disp, SSD1306_BUFFER_SIZE);
#else
	for(uint16_t i = 0; i<SSD1306_BUFFER_SIZE; i++)
	{
		buffer_disp[i] = buffer[i];
	}

	SSD1306_Data(buffer_disp, SSD1306_BUFFER_SIZE);
#endif


}
#ifdef SSD1306_USE_DMA_DOUBLE_BUFFERING
static void XferCpltCallback(DMA_HandleTypeDef *hdma)
{
	if(hdma->Instance == oled_buff_dma->Instance)
	{
		SSD1306_Data(buffer_disp, SSD1306_BUFFER_SIZE);
	}
}
#endif


#ifdef SSD1306_USE_DMA_DOUBLE_BUFFERING
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c, DMA_HandleTypeDef *dma)
{
	oled_i2c = i2c;
	oled_buff_dma = dma;

	if(HAL_I2C_IsDeviceReady(oled_i2c, (SSD1306_ADRESS<<1), OLED_TRIALS, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}


	SSD1306_Command(SSD1306_DISPLAYOFF);
	SSD1306_Command(SSD1306_SETDISPLAYCLOCKDIV);
	SSD1306_Command(0x80);
	SSD1306_Command(SSD1306_SETMULTIPLEX);
	SSD1306_Command(SSD1306_LCDHEIGHT - 1);
	SSD1306_Command(SSD1306_SETDISPLAYOFFSET);
	SSD1306_Command(0x00);
	SSD1306_Command(SSD1306_SETSTARTLINE);
	SSD1306_Command(SSD1306_CHARGEPUMP);
	SSD1306_Command(0x14);

	SSD1306_Command(SSD1306_MEMORYMODE); // 0x20
	SSD1306_Command(0x00); // 0x0 act like ks0108
	SSD1306_Command(SSD1306_SEGREMAP | 0x1);
	SSD1306_Command(SSD1306_COMSCANDEC);

	SSD1306_Command(SSD1306_SETCOMPINS);
	SSD1306_Command(0x12);
	SSD1306_Command(SSD1306_SETCONTRAST);
	SSD1306_Command(0x10);

	SSD1306_Command(SSD1306_SETPRECHARGE); // 0xd9
	SSD1306_Command(0xF1);

	SSD1306_Command(SSD1306_SETVCOMDETECT); // 0xDB
	SSD1306_Command(0x40);
	SSD1306_Command(SSD1306_DISPLAYALLON_RESUME); // 0xA4
	SSD1306_Command(SSD1306_NORMALDISPLAY);       // 0xA6
	SSD1306_Command(SSD1306_DEACTIVATE_SCROLL);

	SSD1306_Command(SSD1306_DISPLAYON);

	/*
	 * Bufforing DMA Cplt Callback
	 */
	oled_buff_dma->XferCpltCallback = XferCpltCallback;
	return HAL_OK;

}
#else
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *i2c)
{
	oled_i2c = i2c;

	if(HAL_I2C_IsDeviceReady(oled_i2c, (SSD1306_ADRESS<<1), OLED_TRIALS, 10) != HAL_OK)
		{
			return HAL_ERROR;
		}

	SSD1306_Command(SSD1306_DISPLAYOFF);
	SSD1306_Command(SSD1306_SETDISPLAYCLOCKDIV);
	SSD1306_Command(0x80);
	SSD1306_Command(SSD1306_SETMULTIPLEX);
	SSD1306_Command(SSD1306_LCDHEIGHT - 1);
	SSD1306_Command(SSD1306_SETDISPLAYOFFSET);
	SSD1306_Command(0x00);
	SSD1306_Command(SSD1306_SETSTARTLINE);
	SSD1306_Command(SSD1306_CHARGEPUMP);
	SSD1306_Command(0x14);

	SSD1306_Command(SSD1306_MEMORYMODE); // 0x20
	SSD1306_Command(0x00); // 0x0 act like ks0108
	SSD1306_Command(SSD1306_SEGREMAP | 0x1);
	SSD1306_Command(SSD1306_COMSCANDEC);

	SSD1306_Command(SSD1306_SETCOMPINS);
	SSD1306_Command(0x12);
	SSD1306_Command(SSD1306_SETCONTRAST);
	SSD1306_Command(0xFF);

	SSD1306_Command(SSD1306_SETPRECHARGE); // 0xd9
	SSD1306_Command(0xF1);

	SSD1306_Command(SSD1306_SETVCOMDETECT); // 0xDB
	SSD1306_Command(0x40);
	SSD1306_Command(SSD1306_DISPLAYALLON_RESUME); // 0xA4
	SSD1306_Command(SSD1306_NORMALDISPLAY);       // 0xA6
	SSD1306_Command(SSD1306_DEACTIVATE_SCROLL);

	SSD1306_Command(SSD1306_DISPLAYON);

	return HAL_OK;
}
#endif
