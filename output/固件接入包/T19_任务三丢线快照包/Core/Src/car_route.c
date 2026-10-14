/* 运动模式、定距和四段路线状态机。出口由里程/航向/稳定边沿共同判定。 */
#include "car_internal.h"
#include <math.h>
#include <string.h>

CarTask3Config CarRoute_Task3Config(void)
{
    CarTask3Config c = CarTask3_DefaultConfig();
    c.start_from_ab = DPT_TASK3_START_FROM_AB;
    c.span_mm = DPT_TASK3_SPAN_MM; c.radius_mm = DPT_TASK3_RADIUS_MM;
    c.arc_mm = DPT_TASK3_ARC_MM;
    c.diagonal_mmps = DPT_TASK3_DIAGONAL_MMPS; c.arc_mmps = DPT_TASK3_ARC_MMPS;
    c.start_mmps = DPT_TASK3_START_MMPS; c.acceleration_mmps2 = DPT_TASK3_ACCEL_MMPS2;
    c.arc_acceleration_mmps2 = DPT_TASK3_ARC_ACCEL_MMPS2;
    c.approach_mm = DPT_TASK3_APPROACH_MM; c.minimum_mmps = DPT_TASK3_MIN_MMPS;
    c.turn_mmps = DPT_TASK3_TURN_MMPS; c.capture_mmps = DPT_TASK3_CAPTURE_MMPS;
    c.deadline_ms = DPT_TASK3_TIMEOUT_MS;
    c.entry_window_mm = DPT_TASK3_ENTRY_WINDOW_MM; c.exit_window_mm = DPT_TASK3_EXIT_WINDOW_MM;
    c.turn_travel_limit_mm = DPT_TASK3_TURN_LIMIT_MM; c.capture_limit_mm = DPT_TASK3_CAPTURE_LIMIT_MM;
    c.align_timeout_ms = DPT_TASK3_TURN_TIMEOUT_MS; c.capture_timeout_ms = DPT_TASK3_CAPTURE_TIMEOUT_MS;
    c.settle_ms = DPT_DISTANCE_SETTLE_MS; c.brake_timeout_ms = DPT_DISTANCE_BRAKE_TIMEOUT_MS;
    return c;
}

int CarRoute_IsDistance(Motion m)
{
    return m == MOVE_DISTANCE || m == MOVE_ROUTE || m == MOVE_LINE || m == MOVE_TASK2 || m == MOVE_TASK3;
}

void CarRoute_Select(CarAppContext *app, Motion selected, uint32_t now)
{
    app->runtime.drive(0, 0);
    CarWheel_Clear(app);
    CarSignal_Set(app, 0);
    app->button.selected_motion = selected;
    memset(&app->route.task3, 0, sizeof app->route.task3);
    memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
    app->route.leg = app->route.edge_seen = 0U;
    app->route.leg_start = 0.0f;
    app->route.elapsed_ms = 0U;
    app->route.motion = MOVE_NONE;
    app->route.tracking = 0U;
    app->route.distance_mm[LEFT] = app->route.distance_mm[RIGHT] = 0.0f;
    app->route.goal = app->button.selected_motion == MOVE_LINE ? DPT_LINE_MM :
        (app->button.selected_motion == MOVE_TASK2 ? 2.0f * (DPT_TASK2_STRAIGHT_MM + DPT_TASK2_ARC_MM) : DPT_ROUTE_MM);
    if (selected == MOVE_TASK3)
        app->route.goal = 2.0f * (hypotf(DPT_TASK3_SPAN_MM, 2.0f * DPT_TASK3_RADIUS_MM) + DPT_TASK3_ARC_MM);
    app->line.center_seen = app->line.center_ready = app->line.invalid_seen = 0U;
    app->line.diff = 0.0f;
    app->ui.button_log_ms = now - 1000U;
    app->ui.render_ms = now - 200U;
}

void CarRoute_Request(CarAppContext *app, Motion requested, uint32_t now)
{
    if (app->runtime.state == BRAKING || app->heading.phase == Y_WARM || app->heading.phase == Y_CAL) return;
    if ((requested == MOVE_HEADING || CarRoute_IsDistance(requested)) && (app->runtime.state == RUNNING || app->heading.phase != Y_READY)) return;
    if (app->runtime.state == RUNNING && (requested == MOVE_TEST || requested == app->route.motion)) return;
    if (requested == MOVE_LINE) {
        CarLine_Decode(app); /* Fresh GPIO gate; never start from an old OLED frame. */
        if (!app->line.center_ready || (uint32_t)(now - app->line.sample_ms) > 15U ||
            !app->line.valid || !app->line.black[1] || !app->line.black[2] || app->line.count != 2U) return;
    }
    if (requested == MOVE_TASK3) {
        if ((uint32_t)(now - app->runtime.old_ms) > 15U ||
            fabsf(app->wheel.items[LEFT].velocity) > 5.0f || fabsf(app->wheel.items[RIGHT].velocity) > 5.0f) return;
        CarLine_Decode(app);
        app->line.sample_ms = now;
    }
    if (!app->runtime.on_seen || (uint32_t)(now - app->runtime.on_ms) < 100U) CarApp_Stop(app, OFF, '-');
    else if (!app->power.valid || (uint32_t)(now - app->power.sample_ms) > 150U) CarApp_Stop(app, ADC_BAD, '-');
    else if (app->power.raw < DPT_MIN_BAT_RAW) CarApp_Stop(app, LOW, '-');
    else if (app->runtime.primed) {
        uint8_t was_running = app->runtime.state == RUNNING;
        app->runtime.drive(0, 0);
        CarWheel_Clear(app);
        CarSignal_Set(app, 0);
        app->route.motion = requested;
        float speed = (requested == MOVE_TEST || requested == MOVE_HEADING || CarRoute_IsDistance(requested)) ? DPT_TARGET_MMPS : (float)app->route.manual_speed;
        app->route.targets[LEFT] = app->route.targets[RIGHT] = speed;
        if (requested == MOVE_BACK) app->route.targets[LEFT] = app->route.targets[RIGHT] = -speed;
        if (requested == MOVE_LEFT) app->route.targets[LEFT] = speed * 0.5f;
        if (requested == MOVE_RIGHT) app->route.targets[RIGHT] = speed * 0.5f;
        if (requested == MOVE_HEADING || CarRoute_IsDistance(requested)) {
            app->heading.reference = app->heading.angle;
            app->heading.error = app->heading.diff = 0.0f;
        }
        app->route.run_duration = (requested == MOVE_TEST || requested == MOVE_HEADING || CarRoute_IsDistance(requested)) ? DPT_DURATION_MS : app->route.manual_duration;
        app->route.tracking = 0;
        if (CarRoute_IsDistance(requested)) {
            for (unsigned i = 0; i < 2U; ++i) {
                app->route.distance_count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(app->wheel.items[i].encoder);
                app->route.distance_mm[i] = 0.0f;
            }
            if (requested == MOVE_TASK2) {
                app->route.leg = app->route.edge_seen = 0U;
                app->route.leg_start = 0.0f;
                app->route.start_heading = app->heading.reference;
                app->route.elapsed_ms = 0U;
            }
            app->route.goal = requested == MOVE_TASK2 ? 2.0f * (DPT_TASK2_STRAIGHT_MM + DPT_TASK2_ARC_MM) : requested == MOVE_ROUTE ? DPT_ROUTE_MM : (requested == MOVE_LINE ? DPT_LINE_MM : DPT_DISTANCE_MM);
            app->route.cruise = requested == MOVE_TASK2 ? DPT_TASK2_STRAIGHT_MMPS : requested == MOVE_ROUTE ? DPT_ROUTE_CRUISE_MMPS : (requested == MOVE_LINE ? DPT_LINE_CRUISE_MMPS : DPT_TARGET_MMPS);
            app->route.slow = requested == MOVE_ROUTE ? DPT_ROUTE_SLOW_MM : DPT_DISTANCE_SLOW_MM;
            app->route.base = requested == MOVE_TASK2 ? DPT_ROUTE_START_MMPS : requested == MOVE_ROUTE ? DPT_ROUTE_START_MMPS : (requested == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_TARGET_MMPS);
            app->line.invalid_seen = 0U;
            app->line.diff = 0.0f;
            app->route.targets[LEFT] = app->route.targets[RIGHT] = app->route.base;
            app->route.tracking = 1;
            app->route.settle_seen = 0;
            app->route.run_duration = requested == MOVE_TASK2 ? DPT_TASK2_TIMEOUT_MS : requested == MOVE_ROUTE ? DPT_ROUTE_TIMEOUT_MS :
                (requested == MOVE_LINE ? DPT_LINE_TIMEOUT_MS : DPT_DISTANCE_TIMEOUT_MS);
        }
        app->route.start_ms = app->route.switch_ms = now;
        app->route.switching = was_running;
        app->runtime.fault_wheel = '-';
        if (requested == MOVE_TASK3) {
            CarTask3Config config = CarRoute_Task3Config();
            CarTask3Input input = CarTask3Port_Input(app, now, 1, 0, 0);
            if (!CarTask3_Start(&app->route.task3, &config, &input)) {
                CarApp_Stop(app, CarTask3Port_StopState(app->route.task3.fault), '-');
                return;
            }
            memset(&app->route.task3_trace, 0, sizeof app->route.task3_trace);
            app->route.leg = 0U; app->route.leg_start = 0.0f; app->route.elapsed_ms = 0U;
            app->route.goal = 2.0f * (app->route.task3.diagonal_mm + config.arc_mm);
            app->route.cruise = config.diagonal_mmps;
            app->route.base = 0.0f;
            app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
            app->route.run_duration = config.deadline_ms;
        }
        app->runtime.state = RUNNING;
    }
}

void CarRoute_UpdateTask2(CarAppContext *app, uint32_t now)
{
    if (app->route.motion != MOVE_TASK2 || app->runtime.state != RUNNING) return;
    float travelled = (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
    float progress = travelled - app->route.leg_start;
    int arc = CarRoute_IsArc(app);
    float length = arc ? DPT_TASK2_ARC_MM : DPT_TASK2_STRAIGHT_MM;
    float window = arc ? DPT_TASK2_EXIT_WINDOW_MM : DPT_TASK2_ENTRY_WINDOW_MM;
    int expected_edge;
    if (!arc) expected_edge = app->line.valid;
    else {
        float end_heading = app->route.leg == 1U ? CarMath_WrapDegrees(app->route.start_heading + 180.0f) : app->route.start_heading;
        float error = CarMath_WrapDegrees(end_heading - app->heading.angle);
        expected_edge = app->line.count == 0U && error >= -DPT_TASK2_END_ANGLE_DEG && error <= DPT_TASK2_END_ANGLE_DEG;
    }
    if (progress >= length - window && progress <= length + window && expected_edge) {
        if (!app->route.edge_seen) { app->route.edge_seen = 1U; app->route.edge_ms = now; }
        if (arc) {
            app->runtime.drive(0, 0); /* Exit-white debounce is stationary, never an open-loop search. */
            CarWheel_Clear(app);
            app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
        }
        if ((uint32_t)(now - app->route.edge_ms) >= DPT_TASK2_EDGE_MS) {
            app->route.edge_seen = app->line.invalid_seen = 0U;
            if (app->route.leg == 3U) { CarApp_Stop(app, BRAKING, '-'); return; }
            ++app->route.leg;
            app->route.leg_start = travelled;
            CarWheel_Clear(app);
            app->signal.ms = now;
            CarSignal_Set(app, 1); /* B, C, D; signalling does not pause the route. */
            app->route.cruise = CarRoute_IsArc(app) ? DPT_LINE_CRUISE_MMPS : DPT_TASK2_STRAIGHT_MMPS;
            if (!CarRoute_IsArc(app)) {
                app->heading.reference = CarMath_WrapDegrees(app->route.start_heading + 180.0f);
                app->heading.error = CarMath_WrapDegrees(app->heading.reference - app->heading.angle);
                app->heading.diff = DPT_HEADING_GAIN * app->heading.error;
                if (app->heading.diff > DPT_HEADING_MAX_DIFF) app->heading.diff = DPT_HEADING_MAX_DIFF;
                if (app->heading.diff < -DPT_HEADING_MAX_DIFF) app->heading.diff = -DPT_HEADING_MAX_DIFF;
            }
            return;
        }
    } else app->route.edge_seen = 0U;
    if (progress > length + window)
        CarApp_Stop(app, arc ? LINE_BAD : LINE_LOST, '-');
}

int CarRoute_IsArc(CarAppContext *app)
{
 return app->route.motion == MOVE_TASK2 && (app->route.leg & 1U); 
}

int CarRoute_UsesLine(CarAppContext *app)
{
 return app->route.motion == MOVE_LINE || CarRoute_IsArc(app); 
}

int CarRoute_UsesHeading(CarAppContext *app)
{
 return app->route.motion != MOVE_TASK3 && app->route.motion != MOVE_LINE && !CarRoute_IsArc(app); 
}

void CarRoute_Update(CarAppContext *app, uint32_t now, const uint16_t count[2])
{
    if (app->route.motion == MOVE_TASK3) {
        if (app->runtime.state != RUNNING || !app->route.tracking) return;
        for (unsigned i = 0; i < 2U; ++i) {
            if (fabsf(app->wheel.items[i].velocity) > 350.0f) {
                CarApp_Stop(app, FAST, i == LEFT ? 'L' : 'R');
                return;
            }
        }
        for (unsigned i = 0; i < 2U; ++i) {
            app->route.distance_mm[i] += CarWheel_Millimetres(i, CarMath_Delta16(count[i], app->route.distance_count[i]));
            app->route.distance_count[i] = count[i];
        }
        int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
        CarTask3Input input = CarTask3Port_Input(app, now, enabled, 0, 0);
        CarTask3Output output;
        if (app->route.task3.phase == T3_ARC && !input.line_valid && !app->route.task3_trace.lost_seen) {
            /* Capture before HALT clears PI/targets. Never read GPIO later as
               evidence of the earlier loss; the car may already be moved. */
            app->route.task3_trace.lost_seen = 1U;
            app->route.task3_trace.lost_ms = now;
            app->route.task3_trace.lost_yaw = input.yaw_deg;
            memcpy(app->route.task3_trace.lost_raw, app->line.raw, 4U);
            for (unsigned w = 0; w < 2U; ++w) {
                app->route.task3_trace.targets[w] = app->route.targets[w];
                app->route.task3_trace.velocity[w] = app->wheel.items[w].velocity;
            }
        }
        CarTask3_Update(&app->route.task3, &input, &output);
        app->route.leg = app->route.task3.leg;
        app->route.leg_start = app->route.task3.leg_start_mm;
        app->route.elapsed_ms = app->route.task3.elapsed_ms;
        app->route.base = output.control == T3_TURN_FORWARD ?
            (output.left_mmps + output.right_mmps) * 0.5f : output.base_mmps;
        if (app->route.task3.phase == T3_FAULT) {
            CarApp_Stop(app, CarTask3Port_StopState(app->route.task3.fault), '-');
        } else if (app->route.task3.phase == T3_DONE) {
            CarApp_Stop(app, DONE, '-');
            app->signal.ms = now;
            CarSignal_Set(app, 1); /* One final A signal, after settled stop. */
        } else CarTask3Port_Apply(app, &output, now);
        if (app->runtime.state == RUNNING && app->route.task3.phase == T3_ARC && input.line_valid) {
            app->route.task3_trace.valid_seen = 1U;
            app->route.task3_trace.lost_seen = 0U; /* Transient recovery starts a fresh loss window. */
            memcpy(app->route.task3_trace.valid_raw, app->line.raw, 4U);
            app->route.task3_trace.valid_error = app->line.error;
            app->route.task3_trace.valid_diff = (app->route.targets[LEFT] - app->route.targets[RIGHT]) * 0.5f;
        }
        return; /* Old distance/Task2 logic must never overwrite Task3 intent. */
    }
                /* Accumulate signed wheel travel only from the fresh start snapshot.
                   Reject timing/implausible counts before declaring distance completion. */
                if (app->route.tracking && (app->runtime.state == RUNNING || app->runtime.state == BRAKING)) {
                    if (app->wheel.items[LEFT].velocity > 350.0f || app->wheel.items[LEFT].velocity < -350.0f)
                        CarApp_Stop(app, FAST, 'L');
                    else if (app->wheel.items[RIGHT].velocity > 350.0f || app->wheel.items[RIGHT].velocity < -350.0f)
                        CarApp_Stop(app, FAST, 'R');
                    else {
                        for (unsigned i = 0; i < 2U; ++i) {
                            app->route.distance_mm[i] += CarWheel_Millimetres(i, CarMath_Delta16(count[i], app->route.distance_count[i]));
                            app->route.distance_count[i] = count[i];
                        }
                        if (app->route.motion == MOVE_TASK2) CarRoute_UpdateTask2(app, now);
                        if (app->route.motion != MOVE_TASK2 && app->runtime.state == RUNNING && (!CarRoute_UsesLine(app) || app->line.valid) && (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f >= app->route.goal)
                            CarApp_Stop(app, BRAKING, '-');
                        if (app->runtime.state == BRAKING) {
                            if (app->wheel.items[LEFT].velocity >= -5.0f && app->wheel.items[LEFT].velocity <= 5.0f &&
                                app->wheel.items[RIGHT].velocity >= -5.0f && app->wheel.items[RIGHT].velocity <= 5.0f) {
                                if (!app->route.settle_seen) { app->route.settle_seen = 1; app->route.settle_ms = now; }
                                else if ((uint32_t)(now - app->route.settle_ms) >= DPT_DISTANCE_SETTLE_MS) CarApp_Stop(app, DONE, '-');
                            } else app->route.settle_seen = 0;
                        }
                    }
                }
                if (app->runtime.state == RUNNING && (app->route.motion == MOVE_HEADING || CarRoute_IsDistance(app->route.motion))) {
                    if (app->heading.phase != Y_READY || (CarRoute_UsesHeading(app) && (app->heading.error > DPT_HEADING_MAX_ERROR || app->heading.error < -DPT_HEADING_MAX_ERROR)))
                        CarApp_Stop(app, YAW_BAD, '-');
                    else {
                        float base = DPT_TARGET_MMPS;
                        if (CarRoute_IsDistance(app->route.motion)) {
                            base = app->route.cruise;
                            float remaining = app->route.motion == MOVE_TASK2 ? DPT_DISTANCE_SLOW_MM : app->route.goal - (app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f;
                            if (remaining < app->route.slow) {
                                base = app->route.cruise * remaining / app->route.slow;
                                if (base < DPT_DISTANCE_MIN_MMPS) base = DPT_DISTANCE_MIN_MMPS;
                            }
                            if (app->route.motion == MOVE_ROUTE || app->route.motion == MOVE_LINE || app->route.motion == MOVE_TASK2) {
                                float ramp = (app->route.motion == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_ROUTE_START_MMPS) + DPT_ROUTE_ACCEL_MMPS2 * ((float)(uint32_t)(now - app->route.start_ms) / 1000.0f);
                                if (base > ramp) base = ramp;
                            }
                            app->route.base = base;
                            float max_diff = DPT_HEADING_MAX_DIFF * base / app->route.cruise;
                            if (app->heading.diff > max_diff) app->heading.diff = max_diff;
                            if (app->heading.diff < -max_diff) app->heading.diff = -max_diff;
                        }
                        if (CarRoute_UsesLine(app)) {
                            if (app->route.motion == MOVE_TASK2 && app->route.edge_seen && app->line.count == 0U) {
                                app->route.targets[LEFT] = app->route.targets[RIGHT] = 0.0f;
                            } else CarLine_SetTargets(app, now, base);
                        }
                        else {
                            app->route.targets[LEFT] = base - app->heading.diff;
                            app->route.targets[RIGHT] = base + app->heading.diff;
                        }
                    }
                }
}
