#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"



void uart2_init(u32 bound);


int USART2_IRQHandler(void);



u8 AT_Command_Capture(u8 uart_recv);
#endif

