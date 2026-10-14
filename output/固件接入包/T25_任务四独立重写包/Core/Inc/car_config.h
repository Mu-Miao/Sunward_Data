#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#include "stm32f1xx_hal.h"

/* All existing calibrated/trial values are retained. DPT_ names remain
 * compatible with prior operation sheets. Edit tuning only in this file. */

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
#define DPT_LINE_MM (400.0f * 3.1415926f)
#define DPT_LINE_CRUISE_MMPS 200.0f
#define DPT_LINE_START_MMPS 50.0f
#define DPT_LINE_GAIN 64.0f
#define DPT_LINE_MAX_DIFF 80.0f
#define DPT_LINE_TIMEOUT_MS 45000U
#define DPT_LINE_CENTER_MS 100U
#define DPT_LINE_INVALID_MS 100U
/* Task 2: AB straight, BC arc, CD straight, DA a rc. No remote operation. */
#define DPT_TASK2_STRAIGHT_MM 1000.0f
#define DPT_TASK2_ARC_MM 1256.0f /* Field: BC full-white exit at encoder mean1256mm; independent half test unchanged. */
#define DPT_TASK2_STRAIGHT_MMPS 200.0f
#define DPT_TASK2_TIMEOUT_MS 45000U /* Development guard; acceptance requires <=30s. */
#define DPT_TASK2_ENTRY_WINDOW_MM 200.0f
#define DPT_TASK2_EXIT_WINDOW_MM 200.0f
#define DPT_TASK2_EDGE_MS 30U
#define DPT_TASK2_END_ANGLE_DEG 20.0f
/* Task 3 geometry and join guards; start A facing B, then gyro turns right to AC. Positive yaw is left. */
#define DPT_TASK3_START_FROM_AB 1U /* User: initial heading A->B. */
#define DPT_TASK3_SPAN_MM 1000.0f
#define DPT_TASK3_RADIUS_MM 400.0f
#define DPT_TASK3_ARC_MM 1256.0f
#define DPT_TASK3_DIAGONAL_MMPS 240.0f
#define DPT_TASK3_ARC_MMPS 200.0f
#define DPT_TASK3_START_MMPS 50.0f
#define DPT_TASK3_ACCEL_MMPS2 150.0f /* Diagonals only. */
#define DPT_TASK3_ARC_ACCEL_MMPS2 100.0f /* Restore the pre-T17 arc entry ramp. */
#define DPT_TASK3_APPROACH_MM 100.0f
#define DPT_TASK3_MIN_MMPS 60.0f /* Capture still uses its own 50mm/s. */
#define DPT_TASK3_TURN_MMPS 80.0f
#define DPT_TASK3_CAPTURE_MMPS 50.0f
#define DPT_TASK3_TIMEOUT_MS 40000U
#define DPT_TASK3_ENTRY_WINDOW_MM 120.0f
#define DPT_TASK3_EXIT_WINDOW_MM 200.0f
#define DPT_TASK3_TURN_LIMIT_MM 80.0f
#define DPT_TASK3_CAPTURE_LIMIT_MM 60.0f
#define DPT_TASK3_TURN_TIMEOUT_MS 4000U
#define DPT_TASK3_CAPTURE_TIMEOUT_MS 2000U
/* Task4 reuses the tested Task3 geometry/speeds; this is a safety watchdog. */
#define DPT_TASK4_LAP_WATCHDOG_MS 80000U
#define DPT_TASK4_LAPS 4U
#define DPT_TASK4_TIMEOUT_MS 160000U
#endif
