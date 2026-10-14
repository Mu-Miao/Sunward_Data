#ifndef CAR_TASK3_H
#define CAR_TASK3_H
#include <stdint.h>

/* 仅在主循环调用的单圈路线核心，不依赖 HAL、PWM、GPIO、串口或全局状态。
 * 左转角度增加；启动前放在 A 点朝 B，由陀螺仪自动对准 AC。 */
typedef enum { T3_IDLE, T3_DIAGONAL, T3_VERTEX_BRAKE, T3_ALIGN,
    T3_CAPTURE, T3_ARC, T3_FINAL_BRAKE, T3_DONE, T3_FAULT, T3_START_ALIGN, T3_START_SETTLE, T3_ALIGN_SETTLE, T3_VERTEX_SEEK } CarTask3Phase;
typedef enum { T3_OK, T3_CONFIG, T3_ABORT, T3_SENSOR, T3_TIMEOUT,
    T3_HEADING, T3_EDGE, T3_LOST, T3_TURN, T3_SETTLE, T3_DISTANCE } CarTask3Fault;
typedef enum { T3_HALT, T3_HOLD_HEADING, T3_TURN_FORWARD,
    T3_FOLLOW_LINE, T3_SEEK_LINE } CarTask3Control;
typedef enum { T3_NO_POINT, T3_POINT_A, T3_POINT_B,
    T3_POINT_C, T3_POINT_D } CarTask3Point;

typedef struct {
    float span_mm, radius_mm, arc_mm;
    uint8_t start_from_ab; /* 1 表示初始 A 朝 B，需要自动转向；0 表示已朝 AC。 */
    float diagonal_mmps, arc_mmps, start_mmps, acceleration_mmps2;
    float arc_acceleration_mmps2; /* 半圆加速度，与斜线段独立配置。 */
    float turn_mmps, capture_mmps;
    float entry_window_mm, exit_window_mm, turn_travel_limit_mm;
    float capture_limit_mm, approach_mm, minimum_mmps;
    float diagonal_tolerance_deg, arc_tolerance_deg, align_tolerance_deg;
    uint32_t deadline_ms, settle_ms, brake_timeout_ms, align_timeout_ms;
    uint32_t capture_timeout_ms, debounce_ms, lost_ms, sample_max_age_ms;
    uint32_t white_seek_ms, vertex_seek_ms; /* 0 关闭对应恢复动作。 */
    float white_seek_mm, white_seek_deg, seek_mmps, vertex_seek_mm;
    uint32_t recovery_ms; /* 0 保留旧的逐帧恢复；非零时需连续有效，原失线预算不重置。 */
} CarTask3Config;

typedef struct {
    uint32_t now_ms, sample_age_ms;
    float distance_mm; /* 左右编码器的有符号平均里程，包含转向对正期间的移动。 */
    float yaw_deg, left_mmps, right_mmps;
    uint8_t heading_ready, line_valid, black_count, centered;
    uint8_t enabled, stop_requested, protection_fault;
} CarTask3Input;

typedef struct {
    CarTask3Control control;
    float heading_deg, base_mmps;
    float left_mmps, right_mmps; /* 仅在前进转向阶段使用的左右轮目标速度。 */
    CarTask3Point signal_point; /* 点位提示仅在一次更新中给出，由上层计时控制声光持续时间。 */
    uint8_t start_signal, reset_controllers;
} CarTask3Output;

typedef struct {
    CarTask3Config config;
    CarTask3Phase phase;
    CarTask3Fault fault;
    uint8_t leg; /* 当前路线段：0 为 AC，1 为 CB，2 为 BD，3 为 DA。 */
    uint8_t edge_seen, settled_seen, lost_seen, pending_start_signal;
    uint32_t start_ms, phase_ms, edge_ms, settled_ms, lost_start_ms;
    uint32_t previous_ms, elapsed_ms, start_settle_ms;
    uint8_t recovery_seen, arc_valid_seen, loss_seek, seek_aligned, resume_active;
    uint32_t seek_forward_ms, resume_ms;
    float lost_start_mm, lost_yaw_deg, seek_forward_mm;
    uint32_t recovery_start_ms;
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
