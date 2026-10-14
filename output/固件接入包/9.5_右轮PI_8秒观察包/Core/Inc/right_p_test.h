#ifndef RIGHT_P_TEST_H
#define RIGHT_P_TEST_H
#include "stm32f1xx_hal.h"

/* Bench tuning values, not measured gains or final driving parameters. */
#define RPT_TARGET_MMPS     60.0f
#define RPT_KP              32.0f /* PWM counts / (mm/s). */
#define RPT_KI              20.0f /* PWM counts / ((mm/s) * s). Initial trial. */
#define RPT_MAX_CMD         1800
#define RPT_DURATION_MS     8000U
#define RPT_RIGHT_CPR       59996.3333f
#define RPT_CIRC_M          0.2042035225f
#define RPT_MIN_BAT_RAW     1200U /* Bench gate ~10.64V, not battery cutoff spec. */

HAL_StatusTypeDef RightPTest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *right_encoder, ADC_HandleTypeDef *battery_adc,
    void (*motor_ab)(int a, int b));
void RightPTest_Poll(uint32_t tick10_count);
#endif
