#ifndef OLED_H
#define OLED_H

#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t x, uint8_t page);
void OLED_WriteString(uint8_t x, uint8_t page, const char *text);
void OLED_DrawFrame(const uint8_t frame[1024]);

#endif
