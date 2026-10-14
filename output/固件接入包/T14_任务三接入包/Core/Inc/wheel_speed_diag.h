#ifndef WHEEL_SPEED_DIAG_H
#define WHEEL_SPEED_DIAG_H

#include "stm32f1xx_hal.h"

/* Initial estimates from the 16:28 hand-turn log; confirm marked full turns. */
#define WHEEL_DIAG_LEFT_CPR    60104.3333f
#define WHEEL_DIAG_RIGHT_CPR   59996.3333f
#define WHEEL_DIAG_CIRC_M      0.2042035225f /* pi * 0.065 m */

/* TIM8 = physical left, TIM4 = physical right. No motor commands here. */
HAL_StatusTypeDef WheelSpeedDiag_Begin(UART_HandleTypeDef *uart,
                                     TIM_HandleTypeDef *left,
                                     TIM_HandleTypeDef *right);
void WheelSpeedDiag_Poll(uint32_t tick10_count);

#endif
