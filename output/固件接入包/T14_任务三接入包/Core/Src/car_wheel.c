/* 轮速测量与保护。左TIM8正、右TIM4负；PI输出提交由CarApp统一完成。 */
#include "car_internal.h"
#include "car_pi.h"

static const float kp[2] = {DPT_LEFT_KP, DPT_RIGHT_KP};
static const float ki[2] = {DPT_LEFT_KI, DPT_RIGHT_KI};
static const float cpr[2] = {DPT_LEFT_CPR, DPT_RIGHT_CPR};
static const int sign[2] = {1, -1}; /* TIM8 left+, TIM4 right-. */
static const char wheel_name[2] = {'L', 'R'};

void CarWheel_Clear(CarAppContext *app)
{
    for (unsigned i = 0; i < 2U; ++i) {
        app->wheel.items[i].command = 0;
        app->wheel.items[i].integral = 0.0f;
        app->wheel.items[i].bad_sign_us = app->wheel.items[i].stall_us = 0U;
    }
}

void CarWheel_Control(CarAppContext *app, unsigned i, float seconds)
{
    Wheel *w = &app->wheel.items[i];
    float direction = app->route.targets[i] >= 0.0f ? 1.0f : -1.0f;
    float measured = direction * w->velocity;
    float target = direction * app->route.targets[i];
    int limit = (app->route.motion == MOVE_TEST || app->route.motion == MOVE_HEADING || CarRoute_IsDistance(app->route.motion)) ? DPT_MAX_CMD : DPT_MANUAL_MAX_CMD;
    if (w->velocity > 350.0f || w->velocity < -350.0f) {
        CarApp_Stop(app, FAST, wheel_name[i]); return;
    }
    if (measured < -5.0f) w->bad_sign_us += app->runtime.dt_us;
    else w->bad_sign_us = 0;
    if (w->bad_sign_us >= 50000U) {
        CarApp_Stop(app, REVERSE, wheel_name[i]); return;
    }
    if (app->route.motion == MOVE_TASK3 && target == 0.0f) {
        w->integral = 0.0f;
        w->command = 0;
        w->stall_us = 0U;
        return; /* FAST and reverse checks above still apply. */
    }
    float out = CarPI_Update(target, measured, kp[i], ki[i], seconds, limit, &w->integral);
    w->command = CarMath_Round(direction * out);
    if (out >= 1200.0f && measured < 5.0f) w->stall_us += app->runtime.dt_us;
    else w->stall_us = 0;
    if (w->stall_us >= 500000U) CarApp_Stop(app, STALL, wheel_name[i]);
}

int CarWheel_ConfigValid(void)
{
    for (unsigned i = 0; i < 2U; ++i)
        if (kp[i] <= 0.0f || ki[i] < 0.0f || ki[i] > 100.0f || cpr[i] <= 0.0f)
            return 0;
    return 1;
}

float CarWheel_Millimetres(unsigned wheel, int32_t counts)
{
    return sign[wheel] * (float)counts * DPT_CIRC_M * 1000.0f / cpr[wheel];
}

void CarWheel_Measure(CarAppContext *app, const uint16_t count[2], float seconds)
{
    for (unsigned i = 0; i < 2U; ++i)
        app->wheel.items[i].velocity = sign[i] * (float)CarMath_Delta16(count[i], app->wheel.items[i].old_count)
            * DPT_CIRC_M * 1000.0f / cpr[i] / seconds;
}
