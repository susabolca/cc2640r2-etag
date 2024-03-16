
#ifndef EPD_2IN13_H
#define EPD_2IN13_H

#include <stdint.h>

// Display resolution
#define EPD_2IN13_WIDTH       128
#define EPD_2IN13_HEIGHT      296

// #define EPD_2IN13_FULL			0
// #define EPD_2IN13_PART			1

void epd_hw_init();
void EPD_Update();

//void EPD_BWR(int width, int height, int left, int top);

// void EPD_2IN13_DisplayPart(UBYTE *Image);
// void EPD_2IN13_DisplayPartBaseImage(UBYTE *Image);

#endif
