#include "car_task3.h"
#include <math.h>
#include <string.h>

static float wrap(float a)
{
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}
static int positive(float a) { return isfinite(a) && a > 0.0f; }
static int bounded_time(uint32_t t) { return t > 0U && t <= 80000U; }
static int config_valid(const CarTask3Config *c)
{
    if (c->start_from_ab > 1U) return 0;
    if (c->white_seek_ms != 0U &&
        (c->white_seek_ms < 30U || c->white_seek_ms > 300U ||
         !positive(c->white_seek_mm) || c->white_seek_mm > 20.0f ||
         !positive(c->white_seek_deg) || c->white_seek_deg > 12.0f ||
         !positive(c->seek_mmps) || c->seek_mmps > 50.0f)) return 0;
    if (c->vertex_seek_ms != 0U &&
        (c->vertex_seek_ms > 3000U || !positive(c->vertex_seek_mm) ||
         c->vertex_seek_mm > 80.0f || c->vertex_seek_mm > c->entry_window_mm ||
         !positive(c->seek_mmps) || c->seek_mmps > 50.0f)) return 0;
    if (!positive(c->span_mm) || !positive(c->radius_mm) || !positive(c->arc_mm) ||
        !positive(c->diagonal_mmps) || !positive(c->arc_mmps) ||
        !positive(c->start_mmps) || !positive(c->acceleration_mmps2) ||
        !positive(c->arc_acceleration_mmps2) ||
        !positive(c->turn_mmps) || !positive(c->capture_mmps) ||
        !positive(c->entry_window_mm) || !positive(c->exit_window_mm) ||
        !positive(c->turn_travel_limit_mm) || !positive(c->capture_limit_mm) ||
        !positive(c->approach_mm) || !positive(c->minimum_mmps) ||
        !positive(c->diagonal_tolerance_deg) || !positive(c->arc_tolerance_deg) ||
        !positive(c->align_tolerance_deg)) return 0;
    float diagonal = hypotf(c->span_mm, 2.0f * c->radius_mm);
    return isfinite(diagonal) && diagonal < 10000.0f &&
        c->entry_window_mm < diagonal * 0.5f && c->exit_window_mm < c->arc_mm * 0.5f &&
        c->approach_mm < diagonal && c->diagonal_mmps <= 250.0f && c->arc_mmps <= 250.0f &&
        c->start_mmps <= c->diagonal_mmps && c->start_mmps <= c->arc_mmps &&
        c->minimum_mmps <= c->diagonal_mmps && c->minimum_mmps <= c->arc_mmps && c->turn_mmps <= 100.0f && c->capture_mmps <= 100.0f &&
        c->diagonal_tolerance_deg <= 20.0f && c->arc_tolerance_deg <= 20.0f &&
        c->align_tolerance_deg <= 5.0f && bounded_time(c->deadline_ms) &&
        bounded_time(c->settle_ms) && bounded_time(c->brake_timeout_ms) &&
        c->brake_timeout_ms > c->settle_ms && bounded_time(c->align_timeout_ms) &&
        bounded_time(c->capture_timeout_ms) && c->debounce_ms >= 30U && c->debounce_ms <= 100U &&
        c->lost_ms >= c->debounce_ms &&
        ((c->recovery_ms == 0U && c->lost_ms <= 100U) ||
         (c->recovery_ms >= 30U && c->recovery_ms <= 100U &&
          c->recovery_ms < c->lost_ms && c->lost_ms <= 250U)) &&
        c->sample_max_age_ms > 0U && c->sample_max_age_ms <= 20U;
}
static int input_valid(const CarTask3Config *c, const CarTask3Input *i)
{
    return isfinite(i->distance_mm) && isfinite(i->yaw_deg) &&
        isfinite(i->left_mmps) && isfinite(i->right_mmps) && i->heading_ready &&
        i->sample_age_ms <= c->sample_max_age_ms && i->black_count <= 4U &&
        (!i->line_valid || (i->black_count > 0U && i->black_count < 4U)) &&
        (!i->centered || (i->line_valid && i->black_count == 2U));
}
static void fault(CarTask3 *r, CarTask3Fault f)
{
    r->phase = T3_FAULT;
    r->fault = f;
    r->pending_start_signal = 0U;
}
static void phase(CarTask3 *r, CarTask3Phase p, const CarTask3Input *i, CarTask3Output *o)
{
    r->phase = p;
    r->phase_ms = i->now_ms;
    r->phase_start_mm = i->distance_mm;
    r->edge_seen = r->settled_seen = r->lost_seen = r->recovery_seen = 0U;
    r->arc_valid_seen = r->loss_seek = r->seek_aligned = r->resume_active = 0U;
    o->control = T3_HALT;
    o->reset_controllers = 1U;
}
static int stable(CarTask3 *r, int matches, uint32_t now)
{
    if (!matches) { r->edge_seen = 0U; return 0; }
    if (!r->edge_seen) { r->edge_seen = 1U; r->edge_ms = now; }
    return (uint32_t)(now - r->edge_ms) >= r->config.debounce_ms;
}
static float diagonal_heading(const CarTask3 *r)
{
    /* 以 AC 方向为起始参考；向 B 的方向为参考角加 alpha，BD 再加 180 度和 alpha。 */
    return wrap(r->start_heading_deg + (r->leg == 2U ? 180.0f + 2.0f * r->diagonal_angle_deg : 0.0f));
}
static float tangent_heading(const CarTask3 *r, int end)
{
    float east = r->start_heading_deg + r->diagonal_angle_deg;
    /* CB 从朝 B 转到朝 A，向左转约 180 度；DA 从朝 A 转到朝 B，向右转约 180 度。 */
    return wrap(east + ((r->leg == 1U ? end : !end) ? 180.0f : 0.0f));
}
static float align_heading(const CarTask3 *r)
{
    return (r->leg & 1U) ? tangent_heading(r, 0) : diagonal_heading(r);
}
static float speed(const CarTask3 *r, const CarTask3Input *i, float cruise, float remaining, float acceleration)
{
    float v = cruise;
    if (remaining < r->config.approach_mm) {
        v *= fmaxf(remaining, 0.0f) / r->config.approach_mm;
        v = fmaxf(v, r->config.minimum_mmps);
    }
    float ramp = r->config.start_mmps + acceleration *
        (float)(uint32_t)(i->now_ms - r->phase_ms) / 1000.0f;
    return fminf(v, ramp);
}

int CarTask3_ConfigValid(const CarTask3Config *c) { return c && config_valid(c); }

CarTask3Config CarTask3_DefaultConfig(void)
{
    CarTask3Config c = {
        .span_mm=1000.0f, .radius_mm=400.0f, .arc_mm=1256.0f, .start_from_ab=1U,
        .diagonal_mmps=200.0f, .arc_mmps=200.0f, .start_mmps=50.0f,
        .acceleration_mmps2=100.0f, .arc_acceleration_mmps2=100.0f, .turn_mmps=60.0f, .capture_mmps=50.0f,
        .entry_window_mm=120.0f, .exit_window_mm=200.0f,
        .turn_travel_limit_mm=80.0f, .capture_limit_mm=60.0f,
        .approach_mm=150.0f, .minimum_mmps=40.0f,
        .diagonal_tolerance_deg=20.0f, .arc_tolerance_deg=20.0f, .align_tolerance_deg=3.0f,
        .deadline_ms=40000U, .settle_ms=200U, .brake_timeout_ms=1500U,
        .align_timeout_ms=4000U, .capture_timeout_ms=2000U,
        .debounce_ms=30U, .lost_ms=100U, .sample_max_age_ms=15U
    };
    return c;
}

int CarTask3_Start(CarTask3 *r, const CarTask3Config *c, const CarTask3Input *i)
{
    if (!r || !c || !i) return 0;
    if (r->phase != T3_IDLE && r->phase != T3_DONE && r->phase != T3_FAULT) return 0;
    if (!config_valid(c)) { fault(r, T3_CONFIG); return 0; }
    if (!input_valid(c, i)) { fault(r, T3_SENSOR); return 0; }
    if (!i->enabled || i->stop_requested || i->protection_fault) { fault(r, T3_ABORT); return 0; }
    if (fabsf(i->left_mmps) > 5.0f || fabsf(i->right_mmps) > 5.0f) { fault(r, T3_SETTLE); return 0; }
    memset(r, 0, sizeof *r);
    r->config = *c;
    r->diagonal_mm = hypotf(c->span_mm, 2.0f * c->radius_mm);
    r->diagonal_angle_deg = atan2f(2.0f * c->radius_mm, c->span_mm) * 57.2957795131f;
    /* 根据已知的初始 AB 方向建立固定 AC 参考，允许陀螺仪当前角度不为零。
       左转角度为正，右转角度为负。 */
    r->start_heading_deg = wrap(i->yaw_deg - (c->start_from_ab ? r->diagonal_angle_deg : 0.0f));
    r->start_ms = r->phase_ms = r->previous_ms = i->now_ms;
    r->leg_start_mm = r->phase_start_mm = i->distance_mm;
    r->phase = c->start_from_ab ? T3_START_ALIGN : T3_DIAGONAL;
    r->pending_start_signal = 1U;
    return 1;
}

void CarTask3_Cancel(CarTask3 *r) { if (r) fault(r, T3_ABORT); }

void CarTask3_Update(CarTask3 *r, const CarTask3Input *i, CarTask3Output *o)
{
    if (!o) return;
    memset(o, 0, sizeof *o); /* 每次更新先设置为停止，避免沿用上次的轮速目标。 */
    if (!r || !i || r->phase == T3_IDLE || r->phase == T3_DONE || r->phase == T3_FAULT) return;
    r->elapsed_ms = (uint32_t)(i->now_ms - r->start_ms);
    if (!i->enabled || i->stop_requested || i->protection_fault) fault(r, T3_ABORT);
    else if (!input_valid(&r->config, i) || (uint32_t)(i->now_ms - r->previous_ms) > 20U) fault(r, T3_SENSOR);
    else if (r->elapsed_ms >= r->config.deadline_ms) fault(r, T3_TIMEOUT);
    r->previous_ms = i->now_ms;
    if (r->phase == T3_FAULT) { o->reset_controllers = 1U; return; }
    if (r->pending_start_signal) {
        o->signal_point = T3_POINT_A;
        o->start_signal = o->reset_controllers = 1U;
        r->pending_start_signal = 0U;
    }
    float progress = i->distance_mm - r->leg_start_mm;
    float phase_travel = i->distance_mm - r->phase_start_mm;
    if (progress < -5.0f || phase_travel < -5.0f) fault(r, T3_DISTANCE);
    else if (r->phase == T3_DIAGONAL) {
        float target = diagonal_heading(r);
        if (fabsf(wrap(target - i->yaw_deg)) > r->config.diagonal_tolerance_deg) fault(r, T3_HEADING);
        else if (progress > r->diagonal_mm + r->config.entry_window_mm) fault(r, T3_EDGE);
        else if (progress >= r->diagonal_mm - r->config.entry_window_mm && i->line_valid) {
            /* 检测到候选顶点时先停止，再静止去抖确认。 */
            o->reset_controllers = 1U;
            if (stable(r, 1, i->now_ms)) {
                o->signal_point = r->leg == 0U ? T3_POINT_C : T3_POINT_D;
                phase(r, T3_VERTEX_BRAKE, i, o);
            }
        } else if (r->config.vertex_seek_ms != 0U && progress >= r->diagonal_mm &&
                   i->black_count == 0U) {
            /* 预计已到 C/D 附近但探头仍全白：保留原顶点距离窗，先停再向弧线切线接近。
               此时不报告经过顶点，也不假定已经在线上。 */
            phase(r, T3_VERTEX_SEEK, i, o);
        } else {
            stable(r, 0, i->now_ms);
            o->control = T3_HOLD_HEADING;
            o->heading_deg = target;
            o->base_mmps = speed(r, i, r->config.diagonal_mmps, r->diagonal_mm - progress, r->config.acceleration_mmps2);
        }
    } else if (r->phase == T3_ARC) {
        int end_white = progress >= r->config.arc_mm - r->config.exit_window_mm &&
            progress <= r->config.arc_mm + r->config.exit_window_mm && i->black_count == 0U &&
            fabsf(wrap(tangent_heading(r, 1) - i->yaw_deg)) <= r->config.arc_tolerance_deg;
        if (progress > r->config.arc_mm + r->config.exit_window_mm) fault(r, T3_EDGE);
        else if (end_white) {
            r->lost_seen = 0U;
            o->reset_controllers = 1U;
            if (stable(r, 1, i->now_ms)) {
                if (r->leg == 3U) phase(r, T3_FINAL_BRAKE, i, o);
                else { o->signal_point = T3_POINT_B; phase(r, T3_VERTEX_BRAKE, i, o); }
            }
        } else {
            stable(r, 0, i->now_ms);
            if (!i->line_valid && !r->lost_seen) {
                r->lost_seen = 1U; r->lost_start_ms = i->now_ms;
                r->lost_start_mm = i->distance_mm; r->lost_yaw_deg = i->yaw_deg;
                r->loss_seek = r->arc_valid_seen && i->black_count == 0U && r->config.white_seek_ms != 0U;
                o->reset_controllers = 1U;
            }
            if (r->lost_seen && !i->line_valid && i->black_count != 0U) r->loss_seek = 0U;
            uint32_t budget = r->loss_seek ? r->config.white_seek_ms : r->config.lost_ms;
            float loss_travel = i->distance_mm - r->lost_start_mm;
            if (r->lost_seen && ((uint32_t)(i->now_ms - r->lost_start_ms) >= budget ||
                (r->loss_seek && (loss_travel < -5.0f || loss_travel >= r->config.white_seek_mm ||
                 fabsf(wrap(i->yaw_deg - r->lost_yaw_deg)) > r->config.white_seek_deg)))) {
                fault(r, T3_LOST);
            } else if (!i->line_valid) {
                r->recovery_seen = 0U;
                if (r->loss_seek) {
                    /* 全白且本弧段有有效历史才允许低速桥接；不把 1111 伪装成有效线。 */
                    o->control = T3_SEEK_LINE;
                    o->base_mmps = r->config.seek_mmps;
                    o->heading_deg = wrap(r->lost_yaw_deg + (r->leg == 1U ? 1.0f : -1.0f) *
                        fmaxf(loss_travel, 0.0f) / r->config.radius_mm * 57.2957795131f);
                } else o->reset_controllers = 1U;
            } else {
                int recovered = !r->lost_seen || r->config.recovery_ms == 0U;
                if (!recovered) {
                    if (!r->recovery_seen) { r->recovery_seen = 1U; r->recovery_start_ms = i->now_ms; }
                    recovered = (uint32_t)(i->now_ms - r->recovery_start_ms) >= r->config.recovery_ms;
                }
                if (!recovered) o->reset_controllers = 1U;
                else {
                    if (r->lost_seen) { r->resume_active = 1U; r->resume_ms = i->now_ms; }
                    r->lost_seen = r->recovery_seen = r->loss_seek = 0U;
                    r->arc_valid_seen = 1U;
                    o->control = T3_FOLLOW_LINE;
                    o->base_mmps = speed(r, i, r->config.arc_mmps, r->config.arc_mm - progress, r->config.arc_acceleration_mmps2);
                    if (r->resume_active) {
                        float ramp = r->config.start_mmps + r->config.arc_acceleration_mmps2 *
                            (float)(uint32_t)(i->now_ms - r->resume_ms) / 1000.0f;
                        o->base_mmps = fminf(o->base_mmps, ramp);
                        if (ramp >= r->config.arc_mmps) r->resume_active = 0U;
                    }
                }
            }
        }
    } else if (r->phase == T3_VERTEX_SEEK) {
        float tangent = wrap(r->start_heading_deg + r->diagonal_angle_deg + (r->leg == 2U ? 180.0f : 0.0f));
        float error = wrap(tangent - i->yaw_deg);
        if (progress > r->diagonal_mm + r->config.entry_window_mm ||
            phase_travel >= r->config.vertex_seek_mm ||
            (uint32_t)(i->now_ms - r->phase_ms) >= r->config.vertex_seek_ms) fault(r, T3_EDGE);
        else if (fabsf(wrap(i->yaw_deg - diagonal_heading(r))) > r->diagonal_angle_deg + 12.0f)
            fault(r, T3_HEADING);
        else if (i->line_valid) {
            o->reset_controllers = 1U;
            if (stable(r, 1, i->now_ms)) {
                o->signal_point = r->leg == 0U ? T3_POINT_C : T3_POINT_D;
                phase(r, T3_VERTEX_BRAKE, i, o);
            }
        } else {
            stable(r, 0, i->now_ms);
            if (i->black_count != 0U) {
                /* 全黑/不连续黑线不能作为搜索方向，保持零输出直到原预算到期。 */
                o->reset_controllers = 1U; r->settled_seen = 0U;
            } else if (!r->seek_aligned) {
                if (fabsf(error) <= r->config.align_tolerance_deg) {
                    o->reset_controllers = 1U;
                    if (fabsf(i->left_mmps) <= 5.0f && fabsf(i->right_mmps) <= 5.0f) {
                        if (!r->settled_seen) { r->settled_seen = 1U; r->settled_ms = i->now_ms; }
                        if ((uint32_t)(i->now_ms - r->settled_ms) >= r->config.settle_ms) {
                            r->seek_aligned = 1U; r->seek_forward_ms = i->now_ms; r->seek_forward_mm = i->distance_mm;
                        }
                    } else r->settled_seen = 0U;
                } else {
                    r->settled_seen = 0U; o->control = T3_TURN_FORWARD; o->heading_deg = tangent;
                    o->left_mmps = error < 0.0f ? r->config.turn_mmps : 0.0f;
                    o->right_mmps = error > 0.0f ? r->config.turn_mmps : 0.0f;
                }
            } else {
                o->control = T3_HOLD_HEADING; o->base_mmps = r->config.seek_mmps;
                float advance = fmaxf(i->distance_mm - r->seek_forward_mm, 0.0f);
                o->heading_deg = wrap(tangent + (r->leg == 0U ? 1.0f : -1.0f) *
                    advance / r->config.radius_mm * 57.2957795131f);
            }
        }
    } else if (r->phase == T3_ALIGN_SETTLE) {
        if ((uint32_t)(i->now_ms - r->phase_ms) >= r->config.align_timeout_ms ||
            phase_travel > r->config.turn_travel_limit_mm) fault(r, T3_TURN);
        else if ((uint32_t)(i->now_ms - r->start_settle_ms) >= r->config.brake_timeout_ms) fault(r, T3_SETTLE);
        else if (fabsf(wrap(align_heading(r) - i->yaw_deg)) > r->config.align_tolerance_deg) {
            r->phase = T3_ALIGN; r->edge_seen = r->settled_seen = 0U;
            o->reset_controllers = 1U;
        } else if (fabsf(i->left_mmps) <= 5.0f && fabsf(i->right_mmps) <= 5.0f) {
            if (!r->settled_seen) { r->settled_seen = 1U; r->settled_ms = i->now_ms; }
            if ((uint32_t)(i->now_ms - r->settled_ms) >= r->config.settle_ms) {
                phase(r, r->leg & 1U ? T3_CAPTURE : T3_DIAGONAL, i, o);
                if (!(r->leg & 1U)) r->leg_start_mm = i->distance_mm;
            }
        } else r->settled_seen = 0U;
    } else if (r->phase == T3_VERTEX_BRAKE || r->phase == T3_FINAL_BRAKE || r->phase == T3_START_SETTLE) {
        if (r->phase == T3_START_SETTLE &&
            ((uint32_t)(i->now_ms - r->phase_ms) >= r->config.align_timeout_ms ||
             phase_travel > r->config.turn_travel_limit_mm)) fault(r, T3_TURN);
        else if ((uint32_t)(i->now_ms - (r->phase == T3_START_SETTLE ? r->start_settle_ms : r->phase_ms)) >= r->config.brake_timeout_ms)
            fault(r, T3_SETTLE);
        else if (r->phase == T3_START_SETTLE &&
            fabsf(wrap(r->start_heading_deg - i->yaw_deg)) > r->config.align_tolerance_deg) {
            /* 重新对正时继续使用本次起始转向的原时间和距离预算。 */
            r->phase = T3_START_ALIGN;
            r->settled_seen = r->edge_seen = 0U;
            o->reset_controllers = 1U;
        }
        else if (fabsf(i->left_mmps) <= 5.0f && fabsf(i->right_mmps) <= 5.0f) {
            if (!r->settled_seen) { r->settled_seen = 1U; r->settled_ms = i->now_ms; }
            if ((uint32_t)(i->now_ms - r->settled_ms) >= r->config.settle_ms) {
                if (r->phase == T3_FINAL_BRAKE) {
                    phase(r, T3_DONE, i, o);
                    o->signal_point = T3_POINT_A;
                } else if (r->phase == T3_START_SETTLE) {
                    /* 转向期间也会产生平均编码器里程；完成对正且两轮停稳后才建立 AC 段的新基线。 */
                    phase(r, T3_DIAGONAL, i, o);
                    r->leg_start_mm = i->distance_mm;
                } else {
                    ++r->leg;
                    phase(r, T3_ALIGN, i, o);
                }
            }
        } else r->settled_seen = 0U;
    } else if (r->phase == T3_ALIGN || r->phase == T3_START_ALIGN) {
        float target = r->phase == T3_START_ALIGN ? r->start_heading_deg : align_heading(r);
        float error = wrap(target - i->yaw_deg);
        if ((uint32_t)(i->now_ms - r->phase_ms) >= r->config.align_timeout_ms ||
            phase_travel > r->config.turn_travel_limit_mm) fault(r, T3_TURN);
        else if (fabsf(error) <= r->config.align_tolerance_deg) {
            o->reset_controllers = 1U;
            if (stable(r, 1, i->now_ms)) {
                if (r->phase == T3_START_ALIGN) {
                    /* 停稳单独计时，转向总时间和距离限制仍从原起点计算。 */
                    uint32_t turn_ms = r->phase_ms;
                    float turn_origin = r->phase_start_mm;
                    phase(r, T3_START_SETTLE, i, o);
                    r->start_settle_ms = i->now_ms;
                    r->phase_ms = turn_ms;
                    r->phase_start_mm = turn_origin;
                }
                else {
                    uint32_t turn_ms = r->phase_ms; float origin = r->phase_start_mm;
                    phase(r, T3_ALIGN_SETTLE, i, o);
                    r->start_settle_ms = i->now_ms; r->phase_ms = turn_ms; r->phase_start_mm = origin;
                }
            }
        } else {
            stable(r, 0, i->now_ms);
            o->control = T3_TURN_FORWARD;
            o->heading_deg = target;
            /* 内轮目标为零、外轮目标为正，采用单轮前进转向，不请求倒车。 */
            o->left_mmps = error < 0.0f ? r->config.turn_mmps : 0.0f;
            o->right_mmps = error > 0.0f ? r->config.turn_mmps : 0.0f;
        }
    } else if (r->phase == T3_CAPTURE) {
        float target = tangent_heading(r, 0);
        if ((uint32_t)(i->now_ms - r->phase_ms) >= r->config.capture_timeout_ms ||
            phase_travel > r->config.capture_limit_mm) fault(r, T3_EDGE);
        else if (fabsf(wrap(target - i->yaw_deg)) > r->config.diagonal_tolerance_deg) fault(r, T3_HEADING);
        else if (stable(r, i->line_valid, i->now_ms)) {
            /* 连续一至三路黑线都可用于巡线反馈。
               如果只接受正中两路黑，车辆正在正常纠偏时也可能因捕获超时停车。 */
            phase(r, T3_ARC, i, o);
            /* 从进入半圆时重新计算里程，排除顶点转向和捕获期间的移动。 */
            r->leg_start_mm = i->distance_mm;
        } else {
            /* 三十毫秒捕获确认期间继续纠偏，第一次读到中心图样时不立即停止。
               无效图样重置稳定计时，并继续在时间和距离限制内沿切线接近。 */
            o->control = i->line_valid ? T3_FOLLOW_LINE : T3_HOLD_HEADING;
            o->heading_deg = target;
            o->base_mmps = r->config.capture_mmps;
        }
    }
    if (r->phase == T3_FAULT) {
        memset(o, 0, sizeof *o);
        o->reset_controllers = 1U;
    }
}

const char *CarTask3_LegName(const CarTask3 *r)
{
    static const char *const names[] = {"AC", "CB", "BD", "DA"};
    return r && r->leg < 4U ? names[r->leg] : "--";
}
