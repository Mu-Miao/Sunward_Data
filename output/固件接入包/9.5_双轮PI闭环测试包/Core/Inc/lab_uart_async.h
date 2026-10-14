#ifndef LAB_UART_ASYNC_H
#define LAB_UART_ASYNC_H
#include "stm32f1xx_hal.h"

/* Exclusive USART1 ownership after the blocking boot messages have completed. */
HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef *uart);
void LabUART_IRQHandler(void);
int LabUART_TryWrite(const char *data, uint16_t length);
int LabUART_TryRead(uint8_t *byte);
int LabUART_TakeRxFault(void);
uint32_t LabUART_RxErrors(void);
uint32_t LabUART_TxDropped(void);
#endif
