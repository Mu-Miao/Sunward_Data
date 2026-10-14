/* OLED文本和串口诊断。保留原日志格式及刷新周期，满发送队列不阻塞运动。 */
#include "car_internal.h"
#include "task_oled.h"
#include <stdio.h>

void CarUI_UpdateOLED(CarAppContext *app, uint32_t now, int enabled)
{
    if ((uint32_t)(now - app->ui.render_ms) >= 200U) {
        static const char *const phases[] = {"NONE", "WARM", "CAL", "READY", "FAIL"};
        static const char *const states[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP", "YAW", "BRAKE", "TIMEOUT", "SETTLE", "LOST", "LINE"};
        const char *title = "TASK 1 A->B";
        if (app->route.motion == MOVE_TASK4 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK4)) title = "TASK 4 SEEK V1";
        else if (app->route.motion == MOVE_TASK3 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK3)) title = "TASK 3 SEEK V1";
        else if (app->route.motion == MOVE_TASK2 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK2)) title = "TASK 2 ONE LAP";
        else if (app->route.motion == MOVE_LINE || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_LINE)) title = "TEST HALF ARC";
        else if (app->route.motion == MOVE_DISTANCE) title = "TEST GO20";
        else if (app->route.motion == MOVE_HEADING) title = "TEST GO";
        else if (app->route.motion == MOVE_TEST) title = "TEST RUN";
        else if (app->route.motion != MOVE_NONE && app->route.motion != MOVE_ROUTE) title = "MANUAL";
        char cal[24], run[24], progress[32];
        int lost_snapshot = CarMotion_IsFigureEight(app->route.motion) && app->route.task3.phase == T3_FAULT &&
            app->route.task3.fault == T3_LOST && app->route.task3_trace.lost_seen;
        unsigned snapshot_page = lost_snapshot ? (unsigned)((uint32_t)(now - app->route.task3_trace.lost_ms) / 2000U) % 2U : 0U;
        (void)snprintf(cal, sizeof cal, "YC %s EN %s", phases[app->heading.phase], enabled ? "ON" : "OFF");
        (void)snprintf(run, sizeof run, "STATE %s", states[app->runtime.state]);
        if (app->route.motion == MOVE_TASK2 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK2)) {
            static const char *const legs[] = {"AB", "BC", "CD", "DA"};
            uint32_t elapsed = app->runtime.state == RUNNING || app->runtime.state == BRAKING ? (uint32_t)(now - app->route.start_ms) : app->route.elapsed_ms;
            (void)snprintf(run, sizeof run, "%s %s T%lu.%lus", states[app->runtime.state], legs[app->route.leg],
                (unsigned long)(elapsed / 1000U), (unsigned long)((elapsed / 100U) % 10U));
        }
        if (CarMotion_IsFigureEight(app->route.motion) || (app->route.motion == MOVE_NONE && CarMotion_IsFigureEight(app->button.selected_motion))) {
            uint32_t elapsed = app->runtime.state == RUNNING ? (uint32_t)(now - app->route.start_ms) : app->route.elapsed_ms;
            const char *segment = app->route.task3.phase == T3_START_ALIGN ? "TURN A" :
                (app->route.task3.phase == T3_START_SETTLE ? "WAIT A" :
                 (app->route.task3.phase == T3_ALIGN_SETTLE ? "WAIT" :
                  (app->route.task3.phase == T3_VERTEX_SEEK ? (app->route.task3.leg == 0U ? "SEEK C" : "SEEK D") :
                   (app->route.task3.loss_seek ? "SEEK" : CarTask3_LegName(&app->route.task3)))));
            if (app->route.motion == MOVE_TASK4 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK4)) {
                unsigned lap = app->route.task4.completed_laps + 1U;
                if (lap > DPT_TASK4_LAPS) lap = DPT_TASK4_LAPS;
                const char *short_segment = app->route.task3.phase == T3_START_ALIGN || app->route.task3.phase == T3_START_SETTLE ? "A" : segment;
                (void)snprintf(run, sizeof run, "%s %u/4 %s T%lu.%lu", states[app->runtime.state], lap, short_segment,
                    (unsigned long)(elapsed / 1000U), (unsigned long)((elapsed / 100U) % 10U));
            } else (void)snprintf(run, sizeof run, "%s %s T%lu.%lus", states[app->runtime.state], segment,
                (unsigned long)(elapsed / 1000U), (unsigned long)((elapsed / 100U) % 10U));
        }
        if (lost_snapshot) {
            if (snapshot_page == 0U) {
                (void)snprintf(cal, sizeof cal, "LF%u%u%u%u PRE%u%u%u%u",
                    (unsigned)app->route.task3_trace.lost_raw[0], (unsigned)app->route.task3_trace.lost_raw[1],
                    (unsigned)app->route.task3_trace.lost_raw[2], (unsigned)app->route.task3_trace.lost_raw[3],
                    (unsigned)app->route.task3_trace.valid_raw[0], (unsigned)app->route.task3_trace.valid_raw[1],
                    (unsigned)app->route.task3_trace.valid_raw[2], (unsigned)app->route.task3_trace.valid_raw[3]);
                (void)snprintf(progress, sizeof progress, "P%u F%u S%d Y%d",
                    (unsigned)app->route.task3.phase, (unsigned)app->route.task3.fault,
                    CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f - app->route.task3.leg_start_mm),
                    CarMath_Round(app->route.task3_trace.lost_yaw));
            } else {
                (void)snprintf(cal, sizeof cal, "E%d C%d PRE%u",
                    CarMath_Round(app->route.task3_trace.valid_error * 100.0f),
                    CarMath_Round(app->route.task3_trace.valid_diff), (unsigned)app->route.task3_trace.valid_seen);
                (void)snprintf(progress, sizeof progress, "T%d/%d V%d/%d",
                    CarMath_Round(app->route.task3_trace.targets[LEFT]), CarMath_Round(app->route.task3_trace.targets[RIGHT]),
                    CarMath_Round(app->route.task3_trace.velocity[LEFT]), CarMath_Round(app->route.task3_trace.velocity[RIGHT]));
            }
        } else if (!enabled) {
            CarLine_Read(app);
            (void)snprintf(progress, sizeof progress, "LF %u %u %u %u",
                (unsigned)app->line.raw[0], (unsigned)app->line.raw[1], (unsigned)app->line.raw[2], (unsigned)app->line.raw[3]);
        } else if (CarMotion_IsFigureEight(app->route.motion) &&
            (app->route.task3.phase == T3_START_ALIGN || app->route.task3.phase == T3_START_SETTLE))
            (void)snprintf(progress, sizeof progress, "START Y%d/%d",
                CarMath_Round(app->heading.angle), CarMath_Round(app->route.task3.start_heading_deg));
        else if (CarMotion_IsFigureEight(app->route.motion) || (app->route.motion == MOVE_NONE && CarMotion_IsFigureEight(app->button.selected_motion)))
            (void)snprintf(progress, sizeof progress, "P%u F%u S%d Y%d",
                (unsigned)app->route.task3.phase, (unsigned)app->route.task3.fault,
                CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f - app->route.task3.leg_start_mm),
                CarMath_Round(app->heading.angle));
        else if (app->route.motion == MOVE_TASK2 || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_TASK2))
            (void)snprintf(progress, sizeof progress, "LF%u%u%u%u S%d",
                (unsigned)app->line.raw[0], (unsigned)app->line.raw[1], (unsigned)app->line.raw[2], (unsigned)app->line.raw[3],
                CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f - app->route.leg_start));
        else if (app->route.motion == MOVE_LINE || (app->route.motion == MOVE_NONE && app->button.selected_motion == MOVE_LINE))
            (void)snprintf(progress, sizeof progress, "LF%u%u%u%u S%d/%d",
                (unsigned)app->line.raw[0], (unsigned)app->line.raw[1], (unsigned)app->line.raw[2], (unsigned)app->line.raw[3],
                CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f), CarMath_Round(DPT_LINE_MM));
        else if (CarRoute_IsDistance(app->route.motion))
            (void)snprintf(progress, sizeof progress, "S %d/%dmm",
                CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f), CarMath_Round(app->route.goal));
        else if (app->route.motion == MOVE_NONE)
            (void)snprintf(progress, sizeof progress, "S 0/1000mm");
        else (void)snprintf(progress, sizeof progress, "V %dmm/s", app->runtime.state == RUNNING ? CarMath_Round(app->route.targets[LEFT]) : 0);
        const char *lines[4] = {title, cal, run, progress};
        (void)TaskOLED_SetLines(lines);
        app->ui.render_ms = now; /* 屏幕上一帧尚未完成时，等下次刷新周期再尝试。 */
    }
    TaskOLED_Poll(now);
    if ((uint32_t)(now - app->ui.oled_log_ms) >= 1000U) {
        char msg[80];
        int n = snprintf(msg, sizeof msg, "OLED TRY=SSD1306 SPI=H1 IO_US=%lu\r\n", (unsigned long)TaskOLED_LastIOUs());
        app->ui.oled_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (app->runtime.bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
    if ((!enabled || app->button.selected_motion == MOVE_LINE || app->route.motion == MOVE_LINE || app->button.selected_motion == MOVE_TASK2 || app->route.motion == MOVE_TASK2 || CarMotion_IsFigureEight(app->button.selected_motion) || CarMotion_IsFigureEight(app->route.motion)) && (uint32_t)(now - app->ui.line_log_ms) >= 1000U) {
        char msg[128];
        int n;
        if (app->button.selected_motion == MOVE_LINE || app->route.motion == MOVE_LINE || app->button.selected_motion == MOVE_TASK2 || app->route.motion == MOVE_TASK2 || CarMotion_IsFigureEight(app->button.selected_motion) || CarMotion_IsFigureEight(app->route.motion))
            n = snprintf(msg, sizeof msg, "LINE RAW=%u%u%u%u E100=%d C=%d VALID=%u CENTER=%u MODE=%s SEG=%u\r\n",
                (unsigned)app->line.raw[0], (unsigned)app->line.raw[1], (unsigned)app->line.raw[2], (unsigned)app->line.raw[3],
                CarMath_Round(app->line.error * 100.0f), CarMath_Round(app->line.diff), (unsigned)app->line.valid, (unsigned)app->line.center_ready,
                app->route.motion == MOVE_TASK4 || app->button.selected_motion == MOVE_TASK4 ? "TASK4" : (CarMotion_IsFigureEight(app->route.motion) || CarMotion_IsFigureEight(app->button.selected_motion) ? "TASK3" : (app->route.motion == MOVE_TASK2 || app->button.selected_motion == MOVE_TASK2 ? "TASK2" : "HALF")), (unsigned)app->route.leg);
        else n = snprintf(msg, sizeof msg, "LINE O1=%u O2=%u O3=%u O4=%u\r\n",
            (unsigned)app->line.raw[0], (unsigned)app->line.raw[1], (unsigned)app->line.raw[2], (unsigned)app->line.raw[3]);
        app->ui.line_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (app->runtime.bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
}

/* 诊断发送拥塞时丢弃日志帧，不阻塞整车控制。 */
void CarUI_Log(CarAppContext *app, uint32_t now, int enabled)
{
    if ((uint32_t)(now - app->ui.motion_log_ms) >= (CarRoute_IsDistance(app->route.motion) ? 400U : 200U)) {
        static const char *const names[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP", "YAW", "BRAKE", "TIMEOUT", "SETTLE", "LOST", "LINE"};
        static const char *const modes[] = {"-", "TEST", "FWD", "BACK", "LEFT", "RIGHT", "HEAD", "DIST", "DIST", "LINE", "TASK2", "TASK3", "TASK4"};
        char msg[256];
        int n = snprintf(msg, sizeof msg,
            "DPI T=%d L=%d R=%d UL=%d UR=%d IL=%d IR=%d BAT=%lu DT=%lu ST=%s F=%c RXE=%lu TXD=%lu M=%s TL=%d TR=%d V=%u SEC=%lu\r\n",
            app->runtime.state == RUNNING ? (CarRoute_IsDistance(app->route.motion) ? CarMath_Round(app->route.base) : ((app->route.motion == MOVE_TEST || app->route.motion == MOVE_HEADING || CarRoute_IsDistance(app->route.motion)) ? CarMath_Round(DPT_TARGET_MMPS) : (int)app->route.manual_speed)) : 0,
            CarMath_Round(app->wheel.items[LEFT].velocity), CarMath_Round(app->wheel.items[RIGHT].velocity),
            app->wheel.items[LEFT].command, app->wheel.items[RIGHT].command,
            CarMath_Round(app->wheel.items[LEFT].integral), CarMath_Round(app->wheel.items[RIGHT].integral),
            (unsigned long)app->power.raw, (unsigned long)app->runtime.dt_us, names[app->runtime.state], app->runtime.fault_wheel,
            (unsigned long)(LabUART_RxErrors() + (app->runtime.bluetooth_on ? LabUART_RxErrorsPort(LAB_UART_BT) : 0U)),
            (unsigned long)(LabUART_TxDropped() + (app->runtime.bluetooth_on ? LabUART_TxDroppedPort(LAB_UART_BT) : 0U)),
            modes[app->route.motion], app->runtime.state == RUNNING ? CarMath_Round(app->route.targets[LEFT]) : 0,
            app->runtime.state == RUNNING ? CarMath_Round(app->route.targets[RIGHT]) : 0, app->route.manual_speed,
            (unsigned long)(app->route.manual_duration / 1000U));
        if (CarRoute_IsDistance(app->route.motion) && n > 1 && n < (int)sizeof msg) {
            n -= 2; /* 先去掉行尾换行符，再追加里程字段。 */
            int added = snprintf(msg + n, sizeof msg - (size_t)n,
                " S=%d SL=%d SR=%d G=%d\r\n",
                CarMath_Round((app->route.distance_mm[LEFT] + app->route.distance_mm[RIGHT]) * 0.5f),
                CarMath_Round(app->route.distance_mm[LEFT]), CarMath_Round(app->route.distance_mm[RIGHT]), CarMath_Round(app->route.goal));
            if (added <= 0 || added >= (int)(sizeof msg - (size_t)n)) n = (int)sizeof msg;
            else n += added;
        }
        if (app->route.motion == MOVE_TASK4 && n > 1 && n < (int)sizeof msg) {
            n -= 2;
            int added = snprintf(msg + n, sizeof msg - (size_t)n, " LAP_DONE=%u LAPS=%u ELAPSED_MS=%lu\r\n",
                (unsigned)app->route.task4.completed_laps, (unsigned)DPT_TASK4_LAPS,
                (unsigned long)app->route.task4.elapsed_ms);
            if (added <= 0 || added >= (int)(sizeof msg - (size_t)n)) n = (int)sizeof msg;
            else n += added;
        }
        if ((app->route.motion == MOVE_ROUTE || app->route.motion == MOVE_LINE || app->route.motion == MOVE_TASK2 || CarMotion_IsFigureEight(app->route.motion)) && n > 1 && n < (int)sizeof msg) {
            n -= 2;
            int added = snprintf(msg + n, sizeof msg - (size_t)n, " SIG=%u\r\n", (unsigned)app->signal.active);
            if (added <= 0 || added >= (int)(sizeof msg - (size_t)n)) n = (int)sizeof msg;
            else n += added;
        }
        app->ui.motion_log_ms = now;
        if (n <= 0 || n >= (int)sizeof msg) CarApp_Stop(app, UART_BAD, '-');
        else {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (app->runtime.bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
        /* 发送队列满时丢弃本条日志，控制和命令接收继续运行。 */
    }

    if (app->heading.available && (uint32_t)(now - app->ui.heading_log_ms) >= 1000U) {
        static const char *const phases[] = {"NONE", "WARM", "CAL", "READY", "FAIL"};
        char msg[128];
        int n = snprintf(msg, sizeof msg,
            "HDG YC=%s N=%lu BZ100=%d Y100=%d E100=%d C=%d IO_US=%lu\r\n",
            phases[app->heading.phase], (unsigned long)app->heading.samples, CarMath_Round(app->heading.bias * 100.0f),
            CarMath_Round(app->heading.angle * 100.0f), CarMath_Round(app->heading.error * 100.0f), CarMath_Round(app->heading.diff),
            (unsigned long)app->heading.io_us);
        app->ui.heading_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (app->runtime.bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
    CarSignal_UpdateReady(app, now);
    if ((uint32_t)(now - app->ui.button_log_ms) >= 1000U) {
        static const char *const actions[] = {"-", "CAL", "START", "STOP", "WAIT", "BLOCK", "MODE"};
        char msg[96];
        int n = snprintf(msg, sizeof msg, "KEY EN=%u RAW=%u ARM=%u EVT=%lu ACT=%s %s\r\n",
            (unsigned)enabled, (unsigned)app->button.raw, (unsigned)app->button.armed,
            (unsigned long)app->button.events, actions[app->button.action], app->button.selected_motion == MOVE_TASK4 ? "TASK=4" : (app->button.selected_motion == MOVE_TASK3 ? "TASK=3" : (app->button.selected_motion == MOVE_TASK2 ? "TASK=2" : (app->button.selected_motion == MOVE_LINE ? "TEST=HALF" : "TASK=1"))));
        app->ui.button_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (app->runtime.bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
}
