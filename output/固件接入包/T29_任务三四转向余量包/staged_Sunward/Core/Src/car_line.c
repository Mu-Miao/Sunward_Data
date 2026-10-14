/* LF04巡线。电气O1..O4排列为物理左→右O4/O3/O2/O1，黑0白1。 */
#include "car_internal.h"

void CarLine_Read(CarAppContext *app)
{
    app->line.raw[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET;
    app->line.raw[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;
    app->line.raw[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET;
    app->line.raw[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
}

void CarLine_Decode(CarAppContext *app)
{
    static const int weight[4] = {-3, -1, 1, 3};
    CarLine_Read(app);
    int sum = 0, first = -1, last = -1;
    app->line.count = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
        app->line.black[i] = app->line.raw[3U - i] == DPT_LINE_BLACK_LEVEL;
        if (app->line.black[i]) {
            if (first < 0) first = (int)i;
            last = (int)i;
            ++app->line.count;
            sum += weight[i];
        }
    }
    app->line.valid = app->line.count > 0U && app->line.count < 4U && last - first + 1 == (int)app->line.count;
    if (app->line.valid) app->line.error = (float)sum / app->line.count;
}

void CarLine_Sample(CarAppContext *app, uint32_t now, int timing_ok)
{
    if (app->button.selected_motion != MOVE_LINE && app->route.motion != MOVE_LINE && app->button.selected_motion != MOVE_TASK2 && app->route.motion != MOVE_TASK2 && !CarMotion_IsFigureEight(app->button.selected_motion) && !CarMotion_IsFigureEight(app->route.motion)) return;
    if (!timing_ok) { app->line.center_seen = app->line.center_ready = 0U; return; }
    CarLine_Decode(app);
    app->line.sample_ms = now;
    if (app->line.valid && app->line.count == 2U && app->line.black[1] && app->line.black[2]) {
        if (!app->line.center_seen) { app->line.center_seen = 1U; app->line.center_ms = now; }
        app->line.center_ready = (uint32_t)(now - app->line.center_ms) >= DPT_LINE_CENTER_MS;
    } else app->line.center_seen = app->line.center_ready = 0U;
}

void CarLine_SetTargets(CarAppContext *app, uint32_t now, float base)
{
    if (!app->line.valid) {
        /* 无有效线时立即输出零；不足 100 毫秒的短暂失线恢复后可以继续。
           故障锁存后必须重新满足启动条件，不自动搜线。 */
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        app->route.targets[LEFT] = app->route.targets[RIGHT] = app->line.diff = 0.0f;
        if (!app->line.invalid_seen) { app->line.invalid_seen = 1U; app->line.invalid_ms = now; }
        else if ((uint32_t)(now - app->line.invalid_ms) >= DPT_LINE_INVALID_MS)
            CarApp_Stop(app, app->line.count == 0U ? LINE_LOST : LINE_BAD, '-');
        return;
    }
    app->line.invalid_seen = 0U;
    /* 仅任务四：探头明显偏到边缘时降速，保留原纠偏方向和差速比例。 */
    if (app->route.motion == MOVE_TASK4 &&
        (app->line.error >= 2.0f || app->line.error <= -2.0f) && base > DPT_TASK4_EDGE_MMPS)
        base = DPT_TASK4_EDGE_MMPS;
    if (app->route.motion == MOVE_TASK4) app->route.base = base;
    app->line.diff = DPT_LINE_GAIN * app->line.error * base / DPT_LINE_CRUISE_MMPS;
    float limit = DPT_LINE_MAX_DIFF * base / DPT_LINE_CRUISE_MMPS;
    if (app->line.diff > limit) app->line.diff = limit;
    if (app->line.diff < -limit) app->line.diff = -limit;
    /* 黑线偏右时提高左轮目标速度，使车向右纠偏；两轮目标均保持前进。 */
    app->route.targets[LEFT] = base + app->line.diff;
    app->route.targets[RIGHT] = base - app->line.diff;
}
