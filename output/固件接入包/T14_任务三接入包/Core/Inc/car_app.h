#ifndef CAR_APP_H
#define CAR_APP_H
#include "stm32f1xx_hal.h"

/* Single vehicle instance. Call only from the main foreground context.
 * Begin after CubeMX peripheral/PWM/encoder initialization; it leaves motors
 * stopped. Poll continuously with the unchanged TIM2 10ms counter.
 * Interrupts only enqueue UART data and increment the timer counter.
 */
HAL_StatusTypeDef CarApp_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b));
HAL_StatusTypeDef CarApp_EnableBluetooth(UART_HandleTypeDef *bluetooth);
void CarApp_EnableHeading(int mpu_initialized);
void CarApp_Poll(uint32_t tick10_count);
#endif
