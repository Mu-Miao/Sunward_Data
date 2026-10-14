/* 电池ADC和低压保护。所有运动故障通过统一停车入口处理。 */
#include "car_internal.h"

/* ADC failure/low battery stops motion before command processing. */
void CarPower_Poll(CarAppContext *app, uint32_t now)
{
    if ((uint32_t)(now - app->power.sample_ms) >= 100U) {
        app->power.sample_ms = now;
        app->power.valid = 0;
        if (HAL_ADC_Start(app->power.adc) == HAL_OK && HAL_ADC_PollForConversion(app->power.adc, 1U) == HAL_OK) {
            app->power.raw = HAL_ADC_GetValue(app->power.adc);
            app->power.valid = HAL_ADC_Stop(app->power.adc) == HAL_OK;
        } else (void)HAL_ADC_Stop(app->power.adc);
        if (app->runtime.state == RUNNING || app->runtime.state == BRAKING) {
            if (!app->power.valid) CarApp_Stop(app, ADC_BAD, '-');
            else if (app->power.raw < DPT_MIN_BAT_RAW) CarApp_Stop(app, LOW, '-');
        }
    }
}
