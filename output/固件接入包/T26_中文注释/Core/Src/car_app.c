/* 整车前台调度与统一停车。保持使能/故障优先级和原10ms处理顺序。 */
#include "car_internal.h"
#include <string.h>
#include "task_oled.h"

static CarAppContext instance;

#ifdef CAR_APP_TEST
CarAppContext *CarApp_TestContext(void) { return &instance; }
#endif

void CarApp_Stop(CarAppContext *app, TestState why, char faulty)
{
    uint8_t completed_route = why == DONE && app->runtime.state == BRAKING && (app->route.motion == MOVE_ROUTE || app->route.motion == MOVE_LINE || app->route.motion == MOVE_TASK2);
    app->runtime.drive(0, 0);
    CarWheel_Clear(app);
    CarSignal_Set(app, 0);
    app->route.switching = 0;
    if (app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) app->heading.phase = Y_NONE;
    if (why != BRAKING) app->route.tracking = 0;
    else {
        app->route.brake_ms = HAL_GetTick();
        app->route.settle_seen = 0;
    }
    if (app->route.motion == MOVE_TASK2 && (app->runtime.state == RUNNING || app->runtime.state == BRAKING))
        app->route.elapsed_ms = (uint32_t)(HAL_GetTick() - app->route.start_ms);
    if (CarMotion_IsFigureEight(app->route.motion) && (app->runtime.state == RUNNING || app->runtime.state == BRAKING)) {
        app->route.elapsed_ms = (uint32_t)(HAL_GetTick() - app->route.start_ms);
        if (app->route.motion == MOVE_TASK4) {
            app->route.task4.elapsed_ms = app->route.elapsed_ms;
            app->route.task4.active = 0U;
        } else app->route.task3.elapsed_ms = app->route.elapsed_ms;
        if (why != DONE && app->route.task3.phase != T3_FAULT) {
            CarTask3_Cancel(&app->route.task3);
            if (why == TIMEOUT) app->route.task3.fault = T3_TIMEOUT;
        }
    }
    app->runtime.state = why;
    app->runtime.fault_wheel = faulty;
    if (completed_route) {
        app->signal.ms = HAL_GetTick();
        CarSignal_Set(app, 1);
    }
}

HAL_StatusTypeDef CarApp_Begin(UART_HandleTypeDef *u,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b))
{
    CarAppContext *app = &instance;

    app->runtime.active = 0;
    CarSignal_Set(app, 0);
    app->signal.ms = 0U;
    app->heading.available = app->heading.cal_requested = 0;
    app->heading.phase = Y_NONE;
    app->route.tracking = app->route.settle_seen = 0;
    app->route.distance_mm[LEFT] = app->route.distance_mm[RIGHT] = 0.0f;
    app->route.base = app->route.cruise = DPT_TARGET_MMPS;
    app->route.goal = DPT_DISTANCE_MM;
    app->route.slow = DPT_DISTANCE_SLOW_MM;
    if (motor_ab == NULL) return HAL_ERROR;
    motor_ab(0, 0);
    if (u == NULL || left_encoder == NULL || right_encoder == NULL ||
        left_encoder == right_encoder || battery_adc == NULL ||
        DPT_MAX_CMD <= 0 || DPT_MAX_CMD > 1800 ||
        DPT_TARGET_MMPS <= 0.0f || DPT_TARGET_MMPS > 100.0f ||
        DPT_KEY_HOLD_MS < 1000U || DPT_KEY_HOLD_MS > 3000U ||
        DPT_LINE_BLACK_LEVEL > 1U || DPT_LINE_MM <= 0.0f ||
        DPT_LINE_CRUISE_MMPS < 40.0f || DPT_LINE_CRUISE_MMPS > 400.0f ||
        DPT_LINE_START_MMPS < 40.0f || DPT_LINE_START_MMPS > DPT_LINE_CRUISE_MMPS ||
        DPT_LINE_GAIN <= 0.0f || DPT_LINE_MAX_DIFF <= 0.0f ||
        DPT_LINE_MAX_DIFF > DPT_LINE_CRUISE_MMPS * 0.5f ||
        DPT_LINE_TIMEOUT_MS < 1000U || DPT_LINE_TIMEOUT_MS > 60000U ||
        DPT_TASK2_STRAIGHT_MM != 1000.0f || DPT_TASK2_ARC_MM < 500.0f || DPT_TASK2_ARC_MM > 2500.0f ||
        DPT_TASK2_STRAIGHT_MMPS < 40.0f || DPT_TASK2_STRAIGHT_MMPS > 250.0f ||
        DPT_TASK2_TIMEOUT_MS < 1000U || DPT_TASK2_TIMEOUT_MS > 60000U ||
        DPT_TASK2_ENTRY_WINDOW_MM < 50.0f || DPT_TASK2_ENTRY_WINDOW_MM > 250.0f ||
        DPT_TASK2_EXIT_WINDOW_MM < 50.0f || DPT_TASK2_EXIT_WINDOW_MM > 250.0f ||
        DPT_TASK2_EDGE_MS < 30U || DPT_TASK2_EDGE_MS > 100U ||
        DPT_TASK2_END_ANGLE_DEG <= 0.0f || DPT_TASK2_END_ANGLE_DEG > 20.0f ||
        DPT_LINE_CENTER_MS < 100U || DPT_LINE_INVALID_MS < 50U || DPT_LINE_INVALID_MS > 100U ||
        DPT_CIRC_M <= 0.0f || DPT_DISTANCE_MM <= 0.0f ||
        DPT_DISTANCE_SLOW_MM <= 0.0f || DPT_DISTANCE_SLOW_MM >= DPT_DISTANCE_MM ||
        DPT_DISTANCE_MIN_MMPS < 40.0f || DPT_DISTANCE_MIN_MMPS > DPT_TARGET_MMPS ||
        DPT_DISTANCE_TIMEOUT_MS < 1000U || DPT_DISTANCE_TIMEOUT_MS > 10000U ||
        DPT_DISTANCE_SETTLE_MS < 100U || DPT_DISTANCE_BRAKE_TIMEOUT_MS <= DPT_DISTANCE_SETTLE_MS ||
        DPT_ROUTE_MM != 1000.0f || DPT_ROUTE_CRUISE_MMPS < DPT_TARGET_MMPS ||
        DPT_ROUTE_CRUISE_MMPS > 150.0f || DPT_ROUTE_START_MMPS < DPT_DISTANCE_MIN_MMPS ||
        DPT_ROUTE_START_MMPS > DPT_ROUTE_CRUISE_MMPS ||
        DPT_ROUTE_ACCEL_MMPS2 <= 0.0f || DPT_ROUTE_ACCEL_MMPS2 > 200.0f ||
        DPT_ROUTE_SLOW_MM <= 0.0f || DPT_ROUTE_SLOW_MM >= DPT_ROUTE_MM ||
        DPT_ROUTE_TIMEOUT_MS < 1000U ||
        DPT_ROUTE_TIMEOUT_MS > 15000U - DPT_DISTANCE_BRAKE_TIMEOUT_MS ||
        DPT_ROUTE_SIGNAL_MS == 0U || DPT_ROUTE_SIGNAL_MS > 1000U ||
        DPT_DURATION_MS == 0U ||
        DPT_DURATION_MS > 3000U || DPT_MANUAL_MMPS < 40U ||
        DPT_MANUAL_MMPS > DPT_MANUAL_MAX_MMPS ||
        DPT_MANUAL_MAX_CMD <= 0 || DPT_MANUAL_MAX_CMD > 3000 ||
        DPT_MANUAL_DURATION_MS < 1000U || DPT_MANUAL_DURATION_MS > 30000U) return HAL_ERROR;
    if (!CarWheel_ConfigValid()) return HAL_ERROR;
    CarTask3Config task3_config = CarRoute_Task3Config();
    if (!CarTask3_ConfigValid(&task3_config)) return HAL_ERROR;
    CarTask3Config task4_config = CarTask4_Config();
    if (!CarTask3_ConfigValid(&task4_config) || DPT_TASK4_LAPS != 4U) return HAL_ERROR;
    app->runtime.hz = HAL_RCC_GetHCLKFreq();
    if (app->runtime.hz < 1000000U) return HAL_ERROR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    if (DWT->CYCCNT == before) return HAL_ERROR;
    TaskOLED_Begin(app->runtime.hz); /* 屏幕上电复位延时在十毫秒控制循环开始前完成。 */
    app->power.adc = battery_adc; app->runtime.drive = motor_ab;
    memset(app->wheel.items, 0, sizeof app->wheel.items);
    app->wheel.items[LEFT].encoder = left_encoder;
    app->wheel.items[RIGHT].encoder = right_encoder;
    app->runtime.state = IDLE;
    app->route.motion = MOVE_NONE;
    memset(&app->route.task3, 0, sizeof app->route.task3);
    memset(&app->route.task4, 0, sizeof app->route.task4);
    memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
    app->route.switching = 0;
    app->route.manual_speed = DPT_MANUAL_MMPS;
    app->route.manual_duration = DPT_MANUAL_DURATION_MS;
    app->route.run_duration = DPT_DURATION_MS;
    app->route.targets[LEFT] = app->route.targets[RIGHT] = DPT_TARGET_MMPS;
    app->runtime.primed = app->power.valid = app->runtime.on_seen = app->runtime.bluetooth_on = 0;
    memset(app->command.inputs, 0, sizeof app->command.inputs);
    app->power.raw = app->runtime.dt_us = 0;
    app->runtime.fault_wheel = '-';
    uint32_t now = HAL_GetTick();
    app->power.sample_ms = now - 100U;
    app->ui.motion_log_ms = now;
    app->button.raw = app->button.stable = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    app->button.press_enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    app->button.armed = 0; /* 上电后也要先稳定松手 30 毫秒，才接受新的按键操作。 */
    app->button.change_ms = app->ui.button_log_ms = now;
    app->button.events = 0U;
    app->button.off_pending = app->button.press_busy = 0U;
    app->button.hold_ms = now;
    app->button.action = KEY_NONE;
    app->ui.render_ms = now - 200U;
    app->ui.oled_log_ms = app->ui.line_log_ms = now;
    memset(app->line.raw, 0, sizeof app->line.raw);
    memset(app->line.black, 0, sizeof app->line.black);
    app->route.leg = app->route.edge_seen = 0U;
    app->route.leg_start = app->route.start_heading = 0.0f;
    app->route.elapsed_ms = 0U;
    app->button.selected_motion = MOVE_ROUTE; /* 上电默认选择独立的一米直行任务。 */
    app->line.valid = app->line.count = app->line.center_seen = app->line.center_ready = app->line.invalid_seen = 0U;
    app->line.center_ms = app->line.sample_ms = app->line.invalid_ms = now;
    app->line.error = app->line.diff = 0.0f;
    if (LabUART_Begin(u) != HAL_OK) return HAL_ERROR;
    if (!LabUART_TryWrite(CarCommand_Banner, sizeof CarCommand_Banner - 1U)) return HAL_ERROR;
    app->command.inputs[LAB_UART_USB].ms = now;
    app->runtime.active = 1;
    return HAL_OK;
}

HAL_StatusTypeDef CarApp_EnableBluetooth(UART_HandleTypeDef *bluetooth)
{
    CarAppContext *app = &instance;

    if (!app->runtime.active || app->runtime.bluetooth_on || app->runtime.state == RUNNING) return HAL_ERROR;
    if (LabUART_BeginPort(LAB_UART_BT, bluetooth) != HAL_OK) return HAL_ERROR;
    memset(&app->command.inputs[LAB_UART_BT], 0, sizeof app->command.inputs[LAB_UART_BT]);
    app->command.inputs[LAB_UART_BT].ms = HAL_GetTick();
    app->runtime.bluetooth_on = 1;
    if (!LabUART_TryWritePort(LAB_UART_BT, CarCommand_Banner, sizeof CarCommand_Banner - 1U)) {
        app->runtime.bluetooth_on = 0;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void CarApp_Poll(uint32_t tick10_count)
{
    CarAppContext *app = &instance;

    if (!app->runtime.active) return;
    uint32_t now = HAL_GetTick();
    int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    uint8_t was_busy = app->runtime.state == RUNNING || app->runtime.state == BRAKING || app->heading.phase == Y_WARM || app->heading.phase == Y_CAL;
    KeyEvent key_pressed = CarButton_Poll(app, now, enabled, was_busy);
    if (!enabled) {
        app->runtime.on_seen = 0;
        if (app->signal.active) CarSignal_Set(app, 0);
        if (app->runtime.state == RUNNING || app->runtime.state == BRAKING) CarApp_Stop(app, OFF, '-');
    } else if (!app->runtime.on_seen) { app->runtime.on_seen = 1; app->runtime.on_ms = now; }
    if (app->runtime.state == RUNNING && (uint32_t)(now - app->route.start_ms) >= app->route.run_duration)
        CarApp_Stop(app, CarRoute_IsDistance(app->route.motion) ? TIMEOUT : DONE, '-');
    if (app->runtime.state == BRAKING && (uint32_t)(now - app->route.brake_ms) >= DPT_DISTANCE_BRAKE_TIMEOUT_MS)
        CarApp_Stop(app, SETTLE_BAD, '-');
    if (app->signal.active && (uint32_t)(now - app->signal.ms) >= DPT_ROUTE_SIGNAL_MS) CarSignal_Set(app, 0);
    if (app->route.switching && (uint32_t)(now - app->route.switch_ms) >= DPT_SWITCH_BRAKE_MS) app->route.switching = 0;

    CarPower_Poll(app, now);

    /* USB 与蓝牙分别组帧，不能拼接两个端口的命令片段。
       两个端口都检查完后才处理启动请求，STOP 和通信故障优先。 */
    Motion requested = MOVE_NONE;
    uint8_t stop_requested = 0;
    CarCommand_Poll(app, now, &requested, &stop_requested);
    CarButton_Handle(app, now, enabled, was_busy, key_pressed, &requested, &stop_requested);
    if (app->heading.cal_requested) {
        if (!stop_requested) CarHeading_BeginCalibration(app, now, enabled);
        app->heading.cal_requested = 0;
        requested = MOVE_NONE; /* 校准请求不保留待执行的运动指令。 */
    }
    if (requested != MOVE_NONE && !stop_requested) CarRoute_Request(app, requested, now);
    if (key_pressed && app->button.action == KEY_START && (app->runtime.state != RUNNING || app->route.motion != app->button.selected_motion))
        app->button.action = KEY_BLOCK;

    if (!app->runtime.primed || tick10_count != app->runtime.old_tick) {
        uint16_t count[2];
        for (unsigned i = 0; i < 2U; ++i)
            count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(app->wheel.items[i].encoder);
        uint32_t cycles = DWT->CYCCNT;
        if (app->runtime.primed) {
            uint32_t elapsed = cycles - app->runtime.old_cycles;
            app->runtime.dt_us = (uint32_t)((uint64_t)elapsed * 1000000U / app->runtime.hz);
            if ((uint32_t)(now - app->runtime.old_ms) > 1000U || elapsed == 0U) {
                CarLine_Sample(app, now, 0);
                if (app->heading.phase != Y_NONE) app->heading.phase = Y_FAIL;
                if (app->runtime.state == RUNNING || app->runtime.state == BRAKING) CarApp_Stop(app, GAP, '-');
                for (unsigned i = 0; i < 2U; ++i) app->wheel.items[i].velocity = 0.0f;
            } else {
                float seconds = (float)elapsed / app->runtime.hz;
                CarWheel_Measure(app, count, seconds);
                int wheels_moved = count[LEFT] != app->wheel.items[LEFT].old_count || count[RIGHT] != app->wheel.items[RIGHT].old_count;
                if (tick10_count - app->runtime.old_tick != 1U || app->runtime.dt_us < 5000U || app->runtime.dt_us > 15000U) {
                    if (app->heading.phase == Y_CAL || app->heading.phase == Y_READY) app->heading.phase = Y_FAIL;
                    if (app->runtime.state == RUNNING || app->runtime.state == BRAKING) CarApp_Stop(app, GAP, '-');
                } else CarHeading_Sample(app, now, enabled, wheels_moved);
                CarLine_Sample(app, now, tick10_count - app->runtime.old_tick == 1U && app->runtime.dt_us >= 5000U && app->runtime.dt_us <= 15000U);
                /* 传感器读写有时间限制；提交电机输出前再次检查物理使能开关。 */
                if ((app->runtime.state == RUNNING || app->runtime.state == BRAKING) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET)
                    CarApp_Stop(app, OFF, '-');
                if ((app->runtime.state == RUNNING || app->runtime.state == BRAKING) &&
                    (app->route.motion == MOVE_HEADING || CarRoute_IsDistance(app->route.motion)) &&
                    (app->heading.phase != Y_READY || (CarRoute_UsesHeading(app) && (app->heading.error > DPT_HEADING_MAX_ERROR || app->heading.error < -DPT_HEADING_MAX_ERROR))))
                    CarApp_Stop(app, YAW_BAD, '-');
                CarRoute_Update(app, now, count);
                if (app->runtime.state == RUNNING) {
                    if (tick10_count - app->runtime.old_tick != 1U || app->runtime.dt_us < 5000U || app->runtime.dt_us > 15000U)
                        CarApp_Stop(app, GAP, '-');
                    else if (!app->route.switching && (!CarRoute_UsesLine(app) || app->line.valid)) {
                        for (unsigned i = 0; i < 2U && app->runtime.state == RUNNING; ++i)
                            CarWheel_Control(app, i, seconds);
                        /* 两轮保护检查都通过后，才统一提交左右轮输出。 */
                        if (app->runtime.state == RUNNING && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET)
                            CarApp_Stop(app, OFF, '-');
                        if (app->runtime.state == RUNNING) app->runtime.drive(app->wheel.items[RIGHT].command, -app->wheel.items[LEFT].command);
                    }
                }
            }
        } else app->runtime.primed = 1;
        for (unsigned i = 0; i < 2U; ++i) app->wheel.items[i].old_count = count[i];
        app->runtime.old_cycles = cycles; app->runtime.old_tick = tick10_count; app->runtime.old_ms = now;
    }

    CarUI_Log(app, now, enabled);
    CarUI_UpdateOLED(app, now, enabled);

}

void CarApp_EnableHeading(int mpu_initialized)
{
    CarHeading_Enable(&instance, mpu_initialized);
}
