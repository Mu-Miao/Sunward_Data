#include "car_task3_port.h"
#include "car_internal.h"
#include <math.h>

CarTask3Input CarTask3Port_Input(const CarAppContext *app, uint32_t now,
    int enabled, int stop_requested, int protection_fault)
{
    CarTask3Input i = {0};
    i.now_ms = now;
    i.sample_age_ms = (uint32_t)(now - app->line.sample_ms);
    i.distance_mm = (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
    i.yaw_deg = app->heading.angle;
    i.left_mmps = app->wheel.items[LEFT].velocity;
    i.right_mmps = app->wheel.items[RIGHT].velocity;
    i.heading_ready = app->heading.phase == Y_READY;
    i.line_valid = app->line.valid;
    i.black_count = app->line.count;
    i.centered = app->line.valid && app->line.count == 2U && app->line.black[1] && app->line.black[2];
    i.enabled = enabled != 0;
    i.stop_requested = stop_requested != 0;
    i.protection_fault = protection_fault != 0;
    return i;
}

void CarTask3Port_Apply(CarAppContext *app, const CarTask3Output *o, uint32_t now)
{
    app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
    /* 整车已锁存故障时，之前计算的运动输出不得重新使电机启动。 */
    if (app->runtime.state != RUNNING) {
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        return;
    }
    if (o->reset_controllers || o->control == T3_HALT) {
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
    }
    if (o->control == T3_HOLD_HEADING) {
        app->heading.reference = o->heading_deg;
        app->heading.error = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        float limit = fminf(DPT_HEADING_MAX_DIFF, o->base_mmps * 0.5f);
        app->heading.diff = fmaxf(-limit, fminf(limit, DPT_HEADING_GAIN * app->heading.error));
        app->route.targets[LEFT] = o->base_mmps - app->heading.diff;
        app->route.targets[RIGHT] = o->base_mmps + app->heading.diff;
    } else if (o->control == T3_TURN_FORWARD) {
        app->heading.reference = o->heading_deg;
        app->heading.error = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        app->route.targets[LEFT] = o->left_mmps;
        app->route.targets[RIGHT] = o->right_mmps;
        /* 任务三、四接近目标角时降低外轮速度，减少不同车架惯性导致的过冲。
           仍按陀螺仪确认角度，并保留原时间、距离及内轮零目标保护。 */
        if (CarMotion_IsFigureEight(app->route.motion)) {
            float slow = app->route.motion == MOVE_TASK4 ? DPT_TASK4_TURN_SLOW_DEG : DPT_TASK3_TURN_SLOW_DEG;
            float floor = app->route.motion == MOVE_TASK4 ? DPT_TASK4_TURN_MIN_MMPS : DPT_TASK3_TURN_MIN_MMPS;
            float scale = fminf(1.0f, fabsf(app->heading.error) / slow);
            for (unsigned w = 0; w < 2U; ++w)
                if (app->route.targets[w] > 0.0f)
                    app->route.targets[w] = fminf(app->route.targets[w],
                        fmaxf(floor, app->route.targets[w] * scale));
            app->route.base = (app->route.targets[LEFT] + app->route.targets[RIGHT]) * 0.5f;
        }
    } else if (o->control == T3_SEEK_LINE) {
        /* 仅核心已授权的弧段全白桥接；不调用正常巡线接口，也不伪造 valid。 */
        if (app->route.task3.phase != T3_ARC || !app->route.task3.loss_seek ||
            !app->route.task3.arc_valid_seen || !app->route.task3_trace.valid_seen ||
            app->line.valid || app->line.count != 0U) {
            app->runtime.drive(0, 0); CarWheel_Clear(app); return;
        }
        float base = fminf(o->base_mmps, 50.0f), limit = base * 0.4f;
        float heading = CarMath_WrapDegrees(o->heading_deg - app->heading.angle);
        float diff = DPT_LINE_GAIN * app->route.task3_trace.valid_error * base / DPT_LINE_CRUISE_MMPS - DPT_HEADING_GAIN * heading;
        diff = fmaxf(-limit, fminf(limit, diff));
        app->route.targets[LEFT] = base + diff; app->route.targets[RIGHT] = base - diff;
        app->route.base = base; app->line.diff = diff;
        app->heading.reference = o->heading_deg; app->heading.error = heading;
    } else if (o->control == T3_FOLLOW_LINE) {
        CarLine_SetTargets(app, now, o->base_mmps);
    }
    if (app->runtime.state == RUNNING && o->signal_point != T3_NO_POINT) {
        app->signal.ms = now;
        CarSignal_Set(app, 1);
    }
}

TestState CarTask3Port_StopState(CarTask3Fault f)
{
    switch (f) {
    case T3_TIMEOUT: return TIMEOUT;
    case T3_SENSOR: case T3_HEADING: case T3_TURN: return YAW_BAD;
    case T3_LOST: return LINE_LOST;
    case T3_SETTLE: return SETTLE_BAD;
    case T3_EDGE: case T3_CONFIG: case T3_DISTANCE: return LINE_BAD;
    case T3_ABORT: default: return STOPPED;
    }
}
