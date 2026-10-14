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
#define DPT_MANUAL_MMPS 120U
#define DPT_MANUAL_MAX_MMPS 300U
#define DPT_MANUAL_MAX_CMD 3000
#define DPT_MANUAL_DURATION_MS 10000U
#define DPT_SWITCH_BRAKE_MS 200U
#define DPT_LEFT_CPR 60104.3333f
#define DPT_RIGHT_CPR 59996.3333f
#define DPT_CIRC_M 0.2042035225f
#define DPT_MIN_BAT_RAW 1200U /* Bench gate, not a battery cutoff specification. */

HAL_StatusTypeDef DualPITest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b));
HAL_StatusTypeDef DualPITest_EnableBluetooth(UART_HandleTypeDef *bluetooth);
/* Trial heading loop: mm/s differential per degree, not angular-rate gain. */
#define DPT_HEADING_GAIN 1.0f
#define DPT_HEADING_MAX_DIFF 15.0f
#define DPT_HEADING_MAX_ERROR 20.0f
/* First distance task: encoder estimate in mm; effective circumference still needs ruler check. */
#define DPT_DISTANCE_MM 200.0f
#define DPT_DISTANCE_TIMEOUT_MS 10000U
#define DPT_DISTANCE_SLOW_MM 40.0f
#define DPT_DISTANCE_MIN_MMPS 40.0f
#define DPT_DISTANCE_SETTLE_MS 200U
#define DPT_DISTANCE_BRAKE_TIMEOUT_MS 1500U
/* First 1m route trial, independently configured from GO20. */
#define DPT_ROUTE_MM 1000.0f
#define DPT_ROUTE_CRUISE_MMPS 120.0f
#define DPT_ROUTE_START_MMPS 40.0f
#define DPT_ROUTE_ACCEL_MMPS2 100.0f
#define DPT_ROUTE_SLOW_MM 100.0f
#define DPT_ROUTE_TIMEOUT_MS 13500U
#define DPT_ROUTE_SIGNAL_MS 500U
/* C10B KEY_U: PA0 active high, stable release required before each press. */
#define DPT_KEY_DEBOUNCE_MS 30U
#define DPT_KEY_HOLD_MS 1500U /* OFF idle only; one selection until stable release. */
/* LF04 field correction: raw black=0, white=1; physical L->R O4/O3/O2/O1.
   Short direction trial only; these gains are not field-accepted arc tuning. */
#define DPT_LINE_BLACK_LEVEL 0U
#define DPT_LINE_MM 200.0f
#define DPT_LINE_CRUISE_MMPS 60.0f
#define DPT_LINE_START_MMPS 40.0f
#define DPT_LINE_GAIN 8.0f
#define DPT_LINE_MAX_DIFF 24.0f
#define DPT_LINE_CENTER_MS 100U
#define DPT_LINE_INVALID_MS 100U
void DualPITest_EnableHeading(int mpu_initialized);
void DualPITest_Poll(uint32_t tick10_count);
#endif
