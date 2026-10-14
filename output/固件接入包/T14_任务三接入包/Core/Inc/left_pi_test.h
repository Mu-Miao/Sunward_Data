#ifndef LEFT_PI_TEST_H
#define LEFT_PI_TEST_H
#include "stm32f1xx_hal.h"

/* Bench tuning values, not measured gains or final driving parameters. */
#define LPT_TARGET_MMPS     60.0f
#define LPT_KP              32.0f /* PWM counts / (mm/s). */
#define LPT_KI              20.0f /* PWM counts / ((mm/s) * s). Initial trial. */
#define LPT_MAX_CMD         1800
#define LPT_DURATION_MS     8000U
#define LPT_LEFT_CPR       60104.3333f
#define LPT_CIRC_M          0.2042035225f
#define LPT_MIN_BAT_RAW     1200U /* Bench gate ~10.64V, not battery cutoff spec. */

HAL_StatusTypeDef LeftPITest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, ADC_HandleTypeDef *battery_adc,
    void (*motor_ab)(int a, int b));
void LeftPITest_Poll(uint32_t tick10_count);
#endif
