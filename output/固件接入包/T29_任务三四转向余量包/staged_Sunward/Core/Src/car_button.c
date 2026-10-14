/* PA0按键消抖与操作意图。OFF短按松手校准，长按选模式，忙时按下停止。 */
#include "car_internal.h"

KeyEvent CarButton_Poll(CarAppContext *app, uint32_t now, int enabled, int busy)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    if (raw != app->button.raw) {
        app->button.raw = raw;
        app->button.change_ms = now;
        if (raw) { app->button.press_enabled = enabled != 0; app->button.press_busy = busy != 0; }
    }
    if (raw && app->button.press_enabled != (enabled != 0)) {
        app->button.armed = app->button.off_pending = 0U;
    }
    if ((uint32_t)(now - app->button.change_ms) < DPT_KEY_DEBOUNCE_MS) return KEY_EVENT_NONE;
    if (!raw) {
        uint8_t short_off = app->button.off_pending && !enabled;
        app->button.stable = app->button.off_pending = 0U;
        app->button.armed = 1U;
        if (short_off) { ++app->button.events; return KEY_EVENT_SHORT; }
        return KEY_EVENT_NONE;
    }
    if (!app->button.stable) {
        app->button.stable = 1U;
        if (!app->button.armed) return KEY_EVENT_NONE;
        app->button.armed = 0U;
        if (!enabled && !busy && !app->button.press_busy) {
            app->button.off_pending = 1U;
            app->button.hold_ms = now;
            return KEY_EVENT_NONE;
        }
        ++app->button.events;
        return KEY_EVENT_SHORT;
    }
    if (app->button.off_pending) {
        if (enabled || busy) app->button.off_pending = 0U;
        else if ((uint32_t)(now - app->button.hold_ms) >= DPT_KEY_HOLD_MS) {
            app->button.off_pending = 0U;
            ++app->button.events;
            return KEY_EVENT_HOLD;
        }
    }
    return KEY_EVENT_NONE;
}

void CarButton_Handle(CarAppContext *app, uint32_t now, int enabled, uint8_t was_busy, KeyEvent key_pressed, Motion *requested, uint8_t *stop_requested)
{
    /* OFF 状态选择模式与启动运动分开处理，不保留待执行的运动。
       串口 STOP、接收故障或忙时按键都不能排队触发重新启动。 */
    if (!enabled && !was_busy && !(*stop_requested) &&
        ((*requested) == MOVE_LINE || (*requested) == MOVE_ROUTE || (*requested) == MOVE_TASK2 || CarMotion_IsFigureEight(*requested))) {
        CarRoute_Select(app, (*requested), now);
        (*requested) = MOVE_NONE;
    }
    if ((*stop_requested)) app->button.off_pending = 0U;
    if (key_pressed) {
        app->button.action = KEY_BLOCK;
        /* 两个串口已扫描完毕，STOP 和接收故障优先于按键启动。 */
        if (!(*stop_requested)) {
            if (was_busy || app->runtime.state == RUNNING || app->runtime.state == BRAKING || app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) {
                CarApp_Stop(app, STOPPED, '-');
                (*stop_requested) = 1;
                app->button.action = KEY_STOP;
            } else if (key_pressed == KEY_EVENT_HOLD && !enabled && !app->heading.cal_requested) {
                CarRoute_Select(app, app->button.selected_motion == MOVE_ROUTE ? MOVE_LINE : (app->button.selected_motion == MOVE_LINE ? MOVE_TASK2 : (app->button.selected_motion == MOVE_TASK2 ? MOVE_TASK3 : (app->button.selected_motion == MOVE_TASK3 ? MOVE_TASK4 : MOVE_ROUTE))), now);
                (*requested) = MOVE_NONE;
                app->button.action = KEY_MODE;
            } else if (key_pressed == KEY_EVENT_SHORT && !enabled && app->heading.available) {
                app->heading.cal_requested = 1;
                app->button.action = KEY_CAL;
            } else if (key_pressed == KEY_EVENT_SHORT && enabled && app->heading.phase == Y_READY && !app->heading.cal_requested) {
                (*requested) = app->button.selected_motion;
                app->button.action = KEY_START;
            } else app->button.action = KEY_WAIT;
        }
        app->ui.button_log_ms = now - 1000U; /* 尽快输出一次按键诊断，发送过程不阻塞控制。 */
    }
}
