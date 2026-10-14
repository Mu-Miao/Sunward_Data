#ifndef LAB_UART_ASYNC_H
#define LAB_UART_ASYNC_H
#include "stm32f1xx_hal.h"

typedef enum { LAB_UART_USB = 0, LAB_UART_BT, LAB_UART_COUNT } LabUART_Port;
/* Each USART exclusively owns its IRQ and queues after blocking boot messages. */
HAL_StatusTypeDef LabUART_BeginPort(LabUART_Port channel, UART_HandleTypeDef *uart);
void LabUART_IRQHandlerPort(LabUART_Port channel);
int LabUART_TryWritePort(LabUART_Port channel, const char *data, uint16_t length);
int LabUART_TryReadPort(LabUART_Port channel, uint8_t *byte);
int LabUART_TakeRxFaultPort(LabUART_Port channel);
uint32_t LabUART_RxErrorsPort(LabUART_Port channel);
uint32_t LabUART_TxDroppedPort(LabUART_Port channel);
/* Keep the original USART1 API for existing callers. */
HAL_StatusTypeDef LabUART_Begin(UART_HandleTypeDef *uart);
void LabUART_IRQHandler(void);
int LabUART_TryWrite(const char *data, uint16_t length);
int LabUART_TryRead(uint8_t *byte);
int LabUART_TakeRxFault(void);
uint32_t LabUART_RxErrors(void);
uint32_t LabUART_TxDropped(void);
#endif
