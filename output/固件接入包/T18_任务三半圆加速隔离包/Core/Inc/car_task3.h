#ifndef CAR_TASK3_H
#define CAR_TASK3_H
#include <stdint.h>

/* Foreground-only route core. No HAL, PWM, GPIO, UART or global state.
 * Yaw increases on a left turn. Place A facing B before Start(); gyro aligns to AC. */
typedef enum { T3_IDLE, T3_DIAGONAL, T3_VERTEX_BRAKE, T3_ALIGN,
    T3_CAPTURE, T3_ARC, T3_FINAL_BRAKE, T3_DONE, T3_FAULT, T3_START_ALIGN, T3_START_SETTLE } CarTask3Phase;
typedef enum { T3_OK, T3_CONFIG, T3_ABORT, T3_SENSOR, T3_TIMEOUT,
    T3_HEADING, T3_EDGE, T3_LOST, T3_TURN, T3_SETTLE, T3_DISTANCE } CarTask3Fault;
typedef enum { T3_HALT, T3_HOLD_HEADING, T3_TURN_FORWARD,
    T3_FOLLOW_LINE } CarTask3Control;
typedef enum { T3_NO_POINT, T3_POINT_A, T3_POINT_B,
    T3_POINT_C, T3_POINT_D } CarTask3Point;

typedef struct {
    float span_mm, radius_mm, arc_mm;
    uint8_t start_from_ab; /* 1: initial nose A->B; 0: already A->C. */
    float diagonal_mmps, arc_mmps, start_mmps, acceleration_mmps2;
    float arc_acceleration_mmps2; /* Independent of diagonal acceleration. */
    float turn_mmps, capture_mmps;
    float entry_window_mm, exit_window_mm, turn_travel_limit_mm;
    float capture_limit_mm, approach_mm, minimum_mmps;
    float diagonal_tolerance_deg, arc_tolerance_deg, align_tolerance_deg;
    uint32_t deadline_ms, settle_ms, brake_timeout_ms, align_timeout_ms;
    uint32_t capture_timeout_ms, debounce_ms, lost_ms, sample_max_age_ms;
} CarTask3Config;

typedef struct {
    uint32_t now_ms, sample_age_ms;
    float distance_mm; /* Signed encoder mean; includes alignment travel. */
    float yaw_deg, left_mmps, right_mmps;
    uint8_t heading_ready, line_valid, black_count, centered;
    uint8_t enabled, stop_requested, protection_fault;
} CarTask3Input;

typedef struct {
    CarTask3Control control;
    float heading_deg, base_mmps;
    float left_mmps, right_mmps; /* Only used by T3_TURN_FORWARD. */
    CarTask3Point signal_point; /* One update pulse; caller times sound/light. */
    uint8_t start_signal, reset_controllers;
} CarTask3Output;

typedef struct {
    CarTask3Config config;
    CarTask3Phase phase;
    CarTask3Fault fault;
    uint8_t leg; /* 0 AC, 1 CB, 2 BD, 3 DA */
    uint8_t edge_seen, settled_seen, lost_seen, pending_start_signal;
    uint32_t start_ms, phase_ms, edge_ms, settled_ms, lost_start_ms;
    uint32_t previous_ms, elapsed_ms, start_settle_ms;
    float diagonal_mm, diagonal_angle_deg, start_heading_deg;
    float leg_start_mm, phase_start_mm;
} CarTask3;

int CarTask3_ConfigValid(const CarTask3Config *config);
CarTask3Config CarTask3_DefaultConfig(void);
int CarTask3_Start(CarTask3 *route, const CarTask3Config *config,
    const CarTask3Input *input);
void CarTask3_Update(CarTask3 *route, const CarTask3Input *input,
    CarTask3Output *output);
void CarTask3_Cancel(CarTask3 *route);
const char *CarTask3_LegName(const CarTask3 *route);
#endif
