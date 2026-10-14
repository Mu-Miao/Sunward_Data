#ifndef DUAL_PI_TEST_H
#define DUAL_PI_TEST_H
#include "stm32f1xx_hal.h"

/* Initial bench values, validated only by the preceding single-wheel tests. */
#define DPT_TARGET_MMPS 60.0f
#define DPT_LEFT_KP 32.0f
#define DPT_LEFT_KI 20.0f
#define DPT_RIGHT_KP 32.0f
#define DPT_RIGHT_KI 20.0f
#define DPT_MAX_CMD 1800
#define DPT_DURATION_MS 3000U
#define DPT_LEFT_CPR 60104.3333f
#define DPT_RIGHT_CPR 59996.3333f
#define DPT_CIRC_M 0.2042035225f
#define DPT_MIN_BAT_RAW 1200U /* Bench gate, not a battery cutoff specification. */

HAL_StatusTypeDef DualPITest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b));
HAL_StatusTypeDef DualPITest_EnableBluetooth(UART_HandleTypeDef *bluetooth);
void DualPITest_Poll(uint32_t tick10_count);
#endif
