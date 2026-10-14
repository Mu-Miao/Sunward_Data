#ifndef LAB_YAW_DIAG_H
#define LAB_YAW_DIAG_H
#include "stm32f1xx_hal.h"

/* Diagnostic only: all calls in foreground, after MPU6050_Init succeeds. */
void YawDiag_Begin(UART_HandleTypeDef *uart);
void YawDiag_Poll(void);
#endif
