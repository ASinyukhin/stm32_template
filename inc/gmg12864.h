#ifndef __GMG12864_H__
#define __GMG12864_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define DISPLAY_H 64
#define DISPLAY_W 128

#define GPIO_WRITE_BIT(bit, level) do {\
	GPIOA->BSRR = ((~(level) & 1) << ((bit) + 16)) | (((level) & 1) << (bit));\
} while (0)

#define DISPLAY_SET_RS(level) GPIO_WRITE_BIT(RS_LINE_NO, level)
#define DISPLAY_RESET(level)  GPIO_WRITE_BIT(RST_LINE_NO, level)

#define DISPLAY_ON            ((uint8_t) 0b10101111)
#define DIPLAY_SET_START_LINE ((uint8_t) 0b01000000)
#define DISPLAY_SET_PAGE_ADDR ((uint8_t) 0b10110000)

void displaySendCmd(uint8_t cmd);

void displaySendData(uint8_t data);

void displaySendMultiple(uint8_t *data, size_t count);

void clearPixBuf();

void displayBlitScreen();

void displayBlitRect(int x, int y, int w, int h);

void displayClear();

void pixbufSetPixel(int x, int y, bool color);

void displayInitHw();

void displayInit();

#endif
