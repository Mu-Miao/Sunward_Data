/* 完成声光与校准指示。非阻塞；信号到期由CarApp轮询处理。 */
#include "car_internal.h"

void CarSignal_Set(CarAppContext *app, int on)
{
    app->signal.active = on != 0;
    app->signal.ready_led_on = app->signal.active;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CarSignal_UpdateReady(CarAppContext *app, uint32_t now)
{
    if (app->signal.active) return;
    int on = 0;
    if (app->runtime.state != RUNNING && app->runtime.state != BRAKING) {
        if (app->heading.phase == Y_READY) on = 1;
        else if (app->heading.phase == Y_WARM || app->heading.phase == Y_CAL)
            on = ((uint32_t)(now - app->heading.warm_ms) / 250U) % 2U == 0U;
        else if (app->heading.phase == Y_FAIL) on = (now / 100U) % 2U == 0U;
    }
    if (app->signal.ready_led_on != (on != 0)) {
        app->signal.ready_led_on = on != 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
}
