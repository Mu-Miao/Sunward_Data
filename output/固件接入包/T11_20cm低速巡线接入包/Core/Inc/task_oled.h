#ifndef TASK_OLED_H
#define TASK_OLED_H
#include <stdint.h>
/* SSD1306 SPI 128x64 trial; no physical identity or ACK detection. */
void TaskOLED_Begin(uint32_t hclk);
int TaskOLED_SetLines(const char *const lines[4]);
void TaskOLED_Poll(uint32_t now);
uint32_t TaskOLED_LastIOUs(void);
#endif
