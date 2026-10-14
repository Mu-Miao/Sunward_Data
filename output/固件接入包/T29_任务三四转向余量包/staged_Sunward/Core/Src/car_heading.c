/* MPU零偏校准和相对航向积分。前台采样，不在中断内进行I2C访问。 */
#include "car_internal.h"
#include "mpu6050.h"

void CarHeading_Enable(CarAppContext *app, int mpu_initialized)
{
    app->heading.available = mpu_initialized != 0;
    app->heading.phase = app->heading.available ? Y_NONE : Y_FAIL;
    app->heading.samples = app->heading.io_us = 0U;
    app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
    app->ui.heading_log_ms = HAL_GetTick();
}

void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled)
{
    if (app->runtime.state == RUNNING || app->runtime.state == BRAKING || enabled || !app->heading.available) return;
    app->runtime.drive(0, 0);
    CarWheel_Clear(app);
    CarSignal_Set(app, 0);
    app->heading.phase = Y_WARM;
    app->heading.warm_ms = now;
    app->heading.samples = 0U;
    app->heading.sum = 0;
    app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
}

void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved)
{
    if (app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) {
        if (enabled || wheels_moved) { app->heading.phase = Y_FAIL; return; }
        if (app->heading.phase == Y_WARM) {
            if ((uint32_t)(now - app->heading.warm_ms) < 2000U) return;
            app->heading.phase = Y_CAL;
        }
    }
    if (app->heading.phase != Y_CAL && app->heading.phase != Y_READY) return;
    MPU6050_Raw raw;
    uint32_t before = DWT->CYCCNT;
    HAL_StatusTypeDef result = MPU6050_ReadRaw(&raw);
    uint32_t cycles = DWT->CYCCNT;
    app->heading.io_us = (uint32_t)((uint64_t)(cycles - before) * 1000000U / app->runtime.hz);
    if (result != HAL_OK || app->heading.io_us > 5000U ||
        raw.gz >= 32760 || raw.gz <= -32760) { app->heading.phase = Y_FAIL; return; }
    if (app->heading.phase == Y_CAL) {
        const int16_t g[3] = {raw.gx, raw.gy, raw.gz};
        for (unsigned i = 0; i < 3U; ++i) {
            if (app->heading.samples == 0U || g[i] < app->heading.min[i]) app->heading.min[i] = g[i];
            if (app->heading.samples == 0U || g[i] > app->heading.max[i]) app->heading.max[i] = g[i];
        }
        app->heading.sum += raw.gz;
        if (++app->heading.samples < 500U) return;
        if ((int32_t)app->heading.max[0] - app->heading.min[0] > 1000 ||
            (int32_t)app->heading.max[1] - app->heading.min[1] > 1000 ||
            (int32_t)app->heading.max[2] - app->heading.min[2] > 300) { app->heading.phase = Y_FAIL; return; }
        app->heading.bias = (float)app->heading.sum / 500.0f;
        app->heading.angle = app->heading.reference = app->heading.error = app->heading.diff = 0.0f;
        app->heading.previous_rate = ((float)raw.gz - app->heading.bias) / 131.0f;
        app->heading.cycles = cycles;
        app->heading.phase = Y_READY;
        return;
    }
    uint32_t elapsed = cycles - app->heading.cycles;
    app->heading.cycles = cycles;
    if ((uint32_t)((uint64_t)elapsed * 1000000U / app->runtime.hz) < 5000U ||
        (uint32_t)((uint64_t)elapsed * 1000000U / app->runtime.hz) > 20000U) {
        app->heading.phase = Y_FAIL; return;
    }
    float rate = ((float)raw.gz - app->heading.bias) / 131.0f;
    app->heading.angle = CarMath_WrapDegrees(app->heading.angle + 0.5f * (rate + app->heading.previous_rate) * ((float)elapsed / app->runtime.hz));
    app->heading.previous_rate = rate;
    app->heading.error = CarMath_WrapDegrees(app->heading.reference - app->heading.angle);
    app->heading.diff = DPT_HEADING_GAIN * app->heading.error;
    if (app->heading.diff > DPT_HEADING_MAX_DIFF) app->heading.diff = DPT_HEADING_MAX_DIFF;
    if (app->heading.diff < -DPT_HEADING_MAX_DIFF) app->heading.diff = -DPT_HEADING_MAX_DIFF;
}
