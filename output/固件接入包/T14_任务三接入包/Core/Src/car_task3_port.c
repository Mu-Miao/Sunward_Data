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
    /* A latched app fault cannot be overridden by a previously computed intent. */
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
