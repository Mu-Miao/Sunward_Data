#include "dual_pi_test.h"
#include "lab_uart_async.h"
#include <stdio.h>
#include <string.h>

typedef enum { IDLE, RUNNING, DONE, OFF, LOW, ADC_BAD, GAP,
               REVERSE, FAST, STALL, UART_BAD, STOPPED } TestState;
enum { LEFT = 0, RIGHT = 1 };
typedef enum { MOVE_NONE, MOVE_TEST, MOVE_FORWARD, MOVE_BACK, MOVE_LEFT, MOVE_RIGHT } Motion;
static Motion motion;
static float targets[2];
static unsigned manual_speed;
static uint32_t manual_duration, run_duration, switch_ms;
static uint8_t switching;
typedef struct {
    TIM_HandleTypeDef *encoder;
    uint16_t old_count;
    float velocity, integral;
    int command;
    uint32_t bad_sign_us, stall_us;
} Wheel;
static Wheel wheels[2];
static const float kp[2] = {DPT_LEFT_KP, DPT_RIGHT_KP};
static const float ki[2] = {DPT_LEFT_KI, DPT_RIGHT_KI};
static const float cpr[2] = {DPT_LEFT_CPR, DPT_RIGHT_CPR};
static const int sign[2] = {1, -1}; /* TIM8 left+, TIM4 right-. */
static const char wheel_name[2] = {'L', 'R'};
static ADC_HandleTypeDef *adc;
static void (*drive)(int, int);
static TestState state;
static uint8_t active, primed, adc_ok, on_seen, bluetooth_on;
typedef struct {
    char text[12];
    unsigned length;
    uint8_t discard;
    uint32_t ms;
} CommandInput;
static CommandInput inputs[LAB_UART_COUNT];
static uint32_t hz, old_cycles, old_tick, old_ms, start_ms;
static uint32_t adc_ms, log_ms, on_ms, bat_raw, dt_us;
static char fault_wheel;
static const char banner[] =
    "DPI: RUN!=60/3s; F!/B!/L!/R!=120/10s; STOP!; V40..200! TIME1..30!; USB/BT IRQ\r\n";

static void clear_controllers(void)
{
    for (unsigned i = 0; i < 2U; ++i) {
        wheels[i].command = 0;
        wheels[i].integral = 0.0f;
        wheels[i].bad_sign_us = wheels[i].stall_us = 0U;
    }
}

static void stop(TestState why, char faulty)
{
    drive(0, 0);
    clear_controllers();
    switching = 0;
    state = why;
    fault_wheel = faulty;
}

static int32_t delta16(uint16_t now, uint16_t before)
{
    uint32_t d = ((uint32_t)now - (uint32_t)before) & 0xffffU;
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

static int rounded(float value)
{
    return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static void request_motion(Motion requested, uint32_t now)
{
    if (state == RUNNING && (requested == MOVE_TEST || requested == motion)) return;
    if (!on_seen || (uint32_t)(now - on_ms) < 100U) stop(OFF, '-');
    else if (!adc_ok || (uint32_t)(now - adc_ms) > 150U) stop(ADC_BAD, '-');
    else if (bat_raw < DPT_MIN_BAT_RAW) stop(LOW, '-');
    else if (primed) {
        uint8_t was_running = state == RUNNING;
        drive(0, 0);
        clear_controllers();
        motion = requested;
        float speed = requested == MOVE_TEST ? DPT_TARGET_MMPS : (float)manual_speed;
        targets[LEFT] = targets[RIGHT] = speed;
        if (requested == MOVE_BACK) targets[LEFT] = targets[RIGHT] = -speed;
        if (requested == MOVE_LEFT) targets[LEFT] = speed * 0.5f;
        if (requested == MOVE_RIGHT) targets[RIGHT] = speed * 0.5f;
        run_duration = requested == MOVE_TEST ? DPT_DURATION_MS : manual_duration;
        start_ms = switch_ms = now;
        switching = was_running;
        fault_wheel = '-';
        state = RUNNING;
    }
}

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

static void accept_command(CommandInput *in, Motion *requested, uint8_t *stop_requested)
{
    in->text[in->length] = '\0';
    if (!in->discard) {
        if (strcmp(in->text, "STOP") == 0 || strcmp(in->text, "S") == 0) {
            stop(STOPPED, '-');
            *stop_requested = 1;
        }
        if (strcmp(in->text, "RUN") == 0) *requested = MOVE_TEST;
        if (strcmp(in->text, "F") == 0) *requested = MOVE_FORWARD;
        if (strcmp(in->text, "B") == 0) *requested = MOVE_BACK;
        if (strcmp(in->text, "L") == 0) *requested = MOVE_LEFT;
        if (strcmp(in->text, "R") == 0) *requested = MOVE_RIGHT;
        unsigned value;
        /* Configuration is accepted only while stopped. */
        if (state != RUNNING && in->text[0] == 'V' &&
            unsigned_value(in->text + 1, &value) && value >= 40U && value <= 200U)
            manual_speed = value;
        if (state != RUNNING && strncmp(in->text, "TIME", 4) == 0 &&
            unsigned_value(in->text + 4, &value) && value >= 1U && value <= 30U)
            manual_duration = value * 1000U;
    }
    in->length = 0;
    in->discard = 0;
}

HAL_StatusTypeDef DualPITest_Begin(UART_HandleTypeDef *u,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b))
{
    active = 0;
    if (motor_ab == NULL) return HAL_ERROR;
    motor_ab(0, 0);
    if (u == NULL || left_encoder == NULL || right_encoder == NULL ||
        left_encoder == right_encoder || battery_adc == NULL ||
        DPT_MAX_CMD <= 0 || DPT_MAX_CMD > 1800 ||
        DPT_TARGET_MMPS <= 0.0f || DPT_TARGET_MMPS > 100.0f ||
        DPT_CIRC_M <= 0.0f || DPT_DURATION_MS == 0U ||
        DPT_DURATION_MS > 3000U || DPT_MANUAL_MMPS < 40U || DPT_MANUAL_MMPS > 200U ||
        DPT_MANUAL_MAX_CMD <= 0 || DPT_MANUAL_MAX_CMD > 3000 ||
        DPT_MANUAL_DURATION_MS < 1000U || DPT_MANUAL_DURATION_MS > 30000U) return HAL_ERROR;
    for (unsigned i = 0; i < 2U; ++i)
        if (kp[i] <= 0.0f || ki[i] < 0.0f || ki[i] > 100.0f || cpr[i] <= 0.0f)
            return HAL_ERROR;
    hz = HAL_RCC_GetHCLKFreq();
    if (hz < 1000000U) return HAL_ERROR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    if (DWT->CYCCNT == before) return HAL_ERROR;
    adc = battery_adc; drive = motor_ab;
    memset(wheels, 0, sizeof wheels);
    wheels[LEFT].encoder = left_encoder;
    wheels[RIGHT].encoder = right_encoder;
    state = IDLE;
    motion = MOVE_NONE;
    switching = 0;
    manual_speed = DPT_MANUAL_MMPS;
    manual_duration = DPT_MANUAL_DURATION_MS;
    run_duration = DPT_DURATION_MS;
    targets[LEFT] = targets[RIGHT] = DPT_TARGET_MMPS;
    primed = adc_ok = on_seen = bluetooth_on = 0;
    memset(inputs, 0, sizeof inputs);
    bat_raw = dt_us = 0;
    fault_wheel = '-';
    uint32_t now = HAL_GetTick();
    adc_ms = now - 100U;
    log_ms = now;
    if (LabUART_Begin(u) != HAL_OK) return HAL_ERROR;
    if (!LabUART_TryWrite(banner, sizeof banner - 1U)) return HAL_ERROR;
    inputs[LAB_UART_USB].ms = now;
    active = 1;
    return HAL_OK;
}

HAL_StatusTypeDef DualPITest_EnableBluetooth(UART_HandleTypeDef *bluetooth)
{
    if (!active || bluetooth_on || state == RUNNING) return HAL_ERROR;
    if (LabUART_BeginPort(LAB_UART_BT, bluetooth) != HAL_OK) return HAL_ERROR;
    memset(&inputs[LAB_UART_BT], 0, sizeof inputs[LAB_UART_BT]);
    inputs[LAB_UART_BT].ms = HAL_GetTick();
    bluetooth_on = 1;
    if (!LabUART_TryWritePort(LAB_UART_BT, banner, sizeof banner - 1U)) {
        bluetooth_on = 0;
        return HAL_ERROR;
    }
    return HAL_OK;
}

static void control_wheel(unsigned i, float seconds)
{
    Wheel *w = &wheels[i];
    float direction = targets[i] >= 0.0f ? 1.0f : -1.0f;
    float measured = direction * w->velocity;
    float target = direction * targets[i];
    int limit = motion == MOVE_TEST ? DPT_MAX_CMD : DPT_MANUAL_MAX_CMD;
    if (w->velocity > 350.0f || w->velocity < -350.0f) {
        stop(FAST, wheel_name[i]); return;
    }
    if (measured < -5.0f) w->bad_sign_us += dt_us;
    else w->bad_sign_us = 0;
    if (w->bad_sign_us >= 50000U) {
        stop(REVERSE, wheel_name[i]); return;
    }
    float error = target - measured;
    float candidate = w->integral + ki[i] * error * seconds;
    float trial = kp[i] * error + candidate;
    if (!((trial > limit && error > 0.0f) ||
          (trial < 0.0f && error < 0.0f))) {
        if (candidate < 0.0f) candidate = 0.0f;
        if (candidate > limit) candidate = (float)limit;
        w->integral = candidate;
    }
    float out = kp[i] * error + w->integral;
    if (out < 0.0f) out = 0.0f;
    if (out > limit) out = (float)limit;
    w->command = rounded(direction * out);
    if (out >= 1200.0f && measured < 5.0f) w->stall_us += dt_us;
    else w->stall_us = 0;
    if (w->stall_us >= 500000U) stop(STALL, wheel_name[i]);
}

void DualPITest_Poll(uint32_t tick10_count)
{
    if (!active) return;
    uint32_t now = HAL_GetTick();
    int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    if (!enabled) {
        on_seen = 0;
        if (state == RUNNING) stop(OFF, '-');
    } else if (!on_seen) { on_seen = 1; on_ms = now; }
    if (state == RUNNING && (uint32_t)(now - start_ms) >= run_duration)
        stop(DONE, '-');
    if (switching && (uint32_t)(now - switch_ms) >= DPT_SWITCH_BRAKE_MS) switching = 0;

    if ((uint32_t)(now - adc_ms) >= 100U) {
        adc_ms = now;
        adc_ok = 0;
        if (HAL_ADC_Start(adc) == HAL_OK && HAL_ADC_PollForConversion(adc, 1U) == HAL_OK) {
            bat_raw = HAL_ADC_GetValue(adc);
            adc_ok = HAL_ADC_Stop(adc) == HAL_OK;
        } else (void)HAL_ADC_Stop(adc);
        if (state == RUNNING) {
            if (!adc_ok) stop(ADC_BAD, '-');
            else if (bat_raw < DPT_MIN_BAT_RAW) stop(LOW, '-');
        }
    }

    /* Independent streams: never join USB fragments with Bluetooth bytes.
       Defer RUN until both streams have been checked, so STOP/fault wins. */
    Motion requested = MOVE_NONE;
    uint8_t stop_requested = 0;
    unsigned port_count = bluetooth_on ? LAB_UART_COUNT : 1U;
    for (unsigned channel = 0; channel < port_count; ++channel) {
        CommandInput *in = &inputs[channel];
        if (LabUART_TakeRxFaultPort((LabUART_Port)channel)) {
            stop(UART_BAD, '-');
            stop_requested = 1;
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
                accept_command(in, &requested, &stop_requested);
            else if (!in->discard && in->length < sizeof in->text - 1U)
                in->text[in->length++] = (char)ch;
            else in->discard = 1;
        }
    }
    if (requested != MOVE_NONE && !stop_requested) request_motion(requested, now);

    if (!primed || tick10_count != old_tick) {
        uint16_t count[2];
        for (unsigned i = 0; i < 2U; ++i)
            count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(wheels[i].encoder);
        uint32_t cycles = DWT->CYCCNT;
        if (primed) {
            uint32_t elapsed = cycles - old_cycles;
            dt_us = (uint32_t)((uint64_t)elapsed * 1000000U / hz);
            if ((uint32_t)(now - old_ms) > 1000U || elapsed == 0U) {
                if (state == RUNNING) stop(GAP, '-');
                for (unsigned i = 0; i < 2U; ++i) wheels[i].velocity = 0.0f;
            } else {
                float seconds = (float)elapsed / hz;
                for (unsigned i = 0; i < 2U; ++i)
                    wheels[i].velocity = sign[i] * (float)delta16(count[i], wheels[i].old_count)
                        * DPT_CIRC_M * 1000.0f / cpr[i] / seconds;
                if (state == RUNNING) {
                    if (tick10_count - old_tick != 1U || dt_us < 5000U || dt_us > 15000U)
                        stop(GAP, '-');
                    else if (!switching) {
                        for (unsigned i = 0; i < 2U && state == RUNNING; ++i)
                            control_wheel(i, seconds);
                        /* Commit both commands only if both wheels passed their checks. */
                        if (state == RUNNING) drive(wheels[RIGHT].command, -wheels[LEFT].command);
                    }
                }
            }
        } else primed = 1;
        for (unsigned i = 0; i < 2U; ++i) wheels[i].old_count = count[i];
        old_cycles = cycles; old_tick = tick10_count; old_ms = now;
    }

    if ((uint32_t)(now - log_ms) >= 200U) {
        static const char *const names[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP"};
        static const char *const modes[] = {"-", "TEST", "FWD", "BACK", "LEFT", "RIGHT"};
        char msg[160];
        int n = snprintf(msg, sizeof msg,
            "DPI T=%d L=%d R=%d UL=%d UR=%d IL=%d IR=%d BAT=%lu DT=%lu ST=%s F=%c RXE=%lu TXD=%lu M=%s TL=%d TR=%d V=%u SEC=%lu\r\n",
            state == RUNNING ? (motion == MOVE_TEST ? rounded(DPT_TARGET_MMPS) : (int)manual_speed) : 0,
            rounded(wheels[LEFT].velocity), rounded(wheels[RIGHT].velocity),
            wheels[LEFT].command, wheels[RIGHT].command,
            rounded(wheels[LEFT].integral), rounded(wheels[RIGHT].integral),
            (unsigned long)bat_raw, (unsigned long)dt_us, names[state], fault_wheel,
            (unsigned long)(LabUART_RxErrors() + (bluetooth_on ? LabUART_RxErrorsPort(LAB_UART_BT) : 0U)),
            (unsigned long)(LabUART_TxDropped() + (bluetooth_on ? LabUART_TxDroppedPort(LAB_UART_BT) : 0U)),
            modes[motion], state == RUNNING ? rounded(targets[LEFT]) : 0,
            state == RUNNING ? rounded(targets[RIGHT]) : 0, manual_speed,
            (unsigned long)(manual_duration / 1000U));
        log_ms = now;
        if (n <= 0 || n >= (int)sizeof msg) stop(UART_BAD, '-');
        else {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
        /* A full TX queue drops this log frame; control/RX keep running. */
    }
}
