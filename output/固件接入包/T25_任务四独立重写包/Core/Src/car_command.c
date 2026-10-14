/* USB与蓝牙命令各自组帧。先扫描完两口，再处理运动请求，STOP优先。 */
#include "car_internal.h"
#include <string.h>

const char CarCommand_Banner[] =
    "DPI KEY_U OFF:short=CAL,long1.5s=mode; READY+ON:press=start; busy:press=STOP; TASK4!/TASK3!/TASK2!/LF20!/GO100!/GO20!/GO!/RUN!/F!/B!/L!/R!/STOP!/CAL! V/TIME\r\n";

static int unsigned_value(const char *text, unsigned *value)
{
    if (*text == '\0') return 0;
    unsigned n = 0;
    for (; *text; ++text) {
        if (*text < '0' || *text > '9' || n > 1000U) return 0;
        n = n * 10U + (unsigned)(*text - '0');
    }
    *value = n;
    return 1;
}

static void CarCommand_Accept(CarAppContext *app, CommandInput *in, Motion *requested, uint8_t *stop_requested)
{
    in->text[in->length] = '\0';
    if (!in->discard) {
        if (strcmp(in->text, "STOP") == 0 || strcmp(in->text, "S") == 0) {
            CarApp_Stop(app, STOPPED, '-');
            *stop_requested = 1;
        }
        if (strcmp(in->text, "CAL") == 0 && app->runtime.state != RUNNING) app->heading.cal_requested = 1;
        if (strcmp(in->text, "TASK4") == 0) *requested = MOVE_TASK4;
        if (strcmp(in->text, "TASK3") == 0) *requested = MOVE_TASK3;
        if (strcmp(in->text, "TASK2") == 0) *requested = MOVE_TASK2;
        if (strcmp(in->text, "LF20") == 0) *requested = MOVE_LINE;
        if (strcmp(in->text, "GO20") == 0) *requested = MOVE_DISTANCE;
        if (strcmp(in->text, "GO100") == 0) *requested = MOVE_ROUTE;
        if (strcmp(in->text, "GO") == 0) *requested = MOVE_HEADING;
        if (strcmp(in->text, "RUN") == 0) *requested = MOVE_TEST;
        if (strcmp(in->text, "F") == 0) *requested = MOVE_FORWARD;
        if (strcmp(in->text, "B") == 0) *requested = MOVE_BACK;
        if (strcmp(in->text, "L") == 0) *requested = MOVE_LEFT;
        if (strcmp(in->text, "R") == 0) *requested = MOVE_RIGHT;
        unsigned value;
        /* Configuration is accepted only while stopped. */
        if (app->runtime.state != RUNNING && in->text[0] == 'V' &&
            unsigned_value(in->text + 1, &value) &&
            value >= 40U && value <= DPT_MANUAL_MAX_MMPS)
            app->route.manual_speed = value;
        if (app->runtime.state != RUNNING && strncmp(in->text, "TIME", 4) == 0 &&
            unsigned_value(in->text + 4, &value) && value >= 1U && value <= 30U)
            app->route.manual_duration = value * 1000U;
    }
    in->length = 0;
    in->discard = 0;
}

/* Check both input streams before accepting any movement request. */
void CarCommand_Poll(CarAppContext *app, uint32_t now, Motion *requested, uint8_t *stop_requested)
{
    unsigned port_count = app->runtime.bluetooth_on ? LAB_UART_COUNT : 1U;
    for (unsigned channel = 0; channel < port_count; ++channel) {
        CommandInput *in = &app->command.inputs[channel];
        if (LabUART_TakeRxFaultPort((LabUART_Port)channel)) {
            CarApp_Stop(app, UART_BAD, '-');
            *stop_requested = 1;
            in->length = 0;
            in->discard = 1;
            in->ms = now;
        }
        if ((in->length != 0U || in->discard) && (uint32_t)(now - in->ms) >= 100U) {
            in->length = 0;
            in->discard = 0;
        }
        for (unsigned i = 0; i < 32U; ++i) {
            uint8_t ch;
            if (!LabUART_TryReadPort((LabUART_Port)channel, &ch)) break;
            in->ms = now;
            if (ch == '!' || ch == '\r' || ch == '\n')
                CarCommand_Accept(app, in, requested, stop_requested);
            else if (!in->discard && in->length < sizeof in->text - 1U)
                in->text[in->length++] = (char)ch;
            else in->discard = 1;
        }
    }
}
