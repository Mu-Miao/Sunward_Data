#include "dual_pi_test.h"
#include "lab_uart_async.h"
#include "mpu6050.h"
#include "task_oled.h"
#include <stdio.h>
#include <string.h>

typedef enum { IDLE, RUNNING, DONE, OFF, LOW, ADC_BAD, GAP,
               REVERSE, FAST, STALL, UART_BAD, STOPPED, YAW_BAD, BRAKING, TIMEOUT, SETTLE_BAD, LINE_LOST, LINE_BAD } TestState;
enum { LEFT = 0, RIGHT = 1 };
typedef enum { MOVE_NONE, MOVE_TEST, MOVE_FORWARD, MOVE_BACK, MOVE_LEFT, MOVE_RIGHT, MOVE_HEADING, MOVE_DISTANCE, MOVE_ROUTE, MOVE_LINE } Motion;
typedef enum { Y_NONE, Y_WARM, Y_CAL, Y_READY, Y_FAIL } YawPhase;
static YawPhase yaw_phase;
static uint8_t yaw_available, cal_requested;
static uint32_t yaw_warm_ms, yaw_cycles, yaw_samples, yaw_io_us, yaw_log_ms;
static int32_t yaw_sum;
static int16_t yaw_min[3], yaw_max[3];
static float yaw_bias, yaw_angle, yaw_previous_rate, yaw_reference, yaw_error, yaw_diff;
static Motion motion;
static float distance_mm[2], distance_base, distance_goal, distance_cruise, distance_slow;
static uint8_t completion_signal;
static uint32_t signal_ms;
typedef enum { KEY_NONE, KEY_CAL, KEY_START, KEY_STOP, KEY_WAIT, KEY_BLOCK, KEY_MODE } KeyAction;
typedef enum { KEY_EVENT_NONE, KEY_EVENT_SHORT, KEY_EVENT_HOLD } KeyEvent;
static KeyAction key_action;
static uint8_t key_raw, key_stable, key_armed, key_press_enabled, ready_led_on;
static uint32_t key_change_ms, key_events, key_log_ms, key_hold_ms;
static uint8_t key_off_pending, key_press_busy;
static uint32_t oled_render_ms, oled_log_ms, line_log_ms;
static uint8_t line_raw[4], line_black[4], line_valid, line_count;
static uint8_t line_center_seen, line_center_ready, line_invalid_seen;
static uint32_t line_center_ms, line_sample_ms, line_invalid_ms;
static float line_error, line_diff;
static Motion key_motion;
static void read_line(void);
static void line_decode(void);
static void line_sample(uint32_t now, int timing_ok);
static uint16_t distance_count[2];
static uint8_t distance_tracking, settle_seen;
static uint32_t brake_ms, settle_ms;
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
    "DPI KEY_U OFF:short=CAL,long1.5s=mode; READY+ON:press=start; busy:press=STOP; LF20!/GO100!/GO20!/GO!/RUN!/F!/B!/L!/R!/STOP!/CAL! V/TIME\r\n";

static void clear_controllers(void)
{
    for (unsigned i = 0; i < 2U; ++i) {
        wheels[i].command = 0;
        wheels[i].integral = 0.0f;
        wheels[i].bad_sign_us = wheels[i].stall_us = 0U;
    }
}

static int is_distance(Motion m)
{
    return m == MOVE_DISTANCE || m == MOVE_ROUTE || m == MOVE_LINE;
}

/* PC2/PC3 active low, PA15 active high; foreground only, no HAL_Delay. */
static void signal_set(int on)
{
    completion_signal = on != 0;
    ready_led_on = completion_signal;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stop(TestState why, char faulty)
{
    uint8_t completed_route = why == DONE && state == BRAKING && (motion == MOVE_ROUTE || motion == MOVE_LINE);
    drive(0, 0);
    clear_controllers();
    signal_set(0);
    switching = 0;
    if (yaw_phase == Y_WARM || yaw_phase == Y_CAL) yaw_phase = Y_NONE;
    if (why != BRAKING) distance_tracking = 0;
    else {
        brake_ms = HAL_GetTick();
        settle_seen = 0;
    }
    state = why;
    fault_wheel = faulty;
    if (completed_route) {
        signal_ms = HAL_GetTick();
        signal_set(1);
    }
}

/* OFF idle short press is decided at stable RELEASE, so a selection hold
   cannot start calibration first. ON and busy presses still act at 30ms.
   Boot-held and enable changes while held require a fresh stable release. */
static KeyEvent key_poll(uint32_t now, int enabled, int busy)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    if (raw != key_raw) {
        key_raw = raw;
        key_change_ms = now;
        if (raw) { key_press_enabled = enabled != 0; key_press_busy = busy != 0; }
    }
    if (raw && key_press_enabled != (enabled != 0)) {
        key_armed = key_off_pending = 0U;
    }
    if ((uint32_t)(now - key_change_ms) < DPT_KEY_DEBOUNCE_MS) return KEY_EVENT_NONE;
    if (!raw) {
        uint8_t short_off = key_off_pending && !enabled;
        key_stable = key_off_pending = 0U;
        key_armed = 1U;
        if (short_off) { ++key_events; return KEY_EVENT_SHORT; }
        return KEY_EVENT_NONE;
    }
    if (!key_stable) {
        key_stable = 1U;
        if (!key_armed) return KEY_EVENT_NONE;
        key_armed = 0U;
        if (!enabled && !busy && !key_press_busy) {
            key_off_pending = 1U;
            key_hold_ms = now;
            return KEY_EVENT_NONE;
        }
        ++key_events;
        return KEY_EVENT_SHORT;
    }
    if (key_off_pending) {
        if (enabled || busy) key_off_pending = 0U;
        else if ((uint32_t)(now - key_hold_ms) >= DPT_KEY_HOLD_MS) {
            key_off_pending = 0U;
            ++key_events;
            return KEY_EVENT_HOLD;
        }
    }
    return KEY_EVENT_NONE;
}

/* Selection never stores a movement request; both choices have their own title. */
static void select_key_motion(Motion selected, uint32_t now)
{
    drive(0, 0);
    clear_controllers();
    signal_set(0);
    key_motion = selected;
    motion = MOVE_NONE;
    distance_tracking = 0U;
    distance_mm[LEFT] = distance_mm[RIGHT] = 0.0f;
    distance_goal = key_motion == MOVE_LINE ? DPT_LINE_MM : DPT_ROUTE_MM;
    line_center_seen = line_center_ready = line_invalid_seen = 0U;
    line_diff = 0.0f;
    key_log_ms = now - 1000U;
    oled_render_ms = now - 200U;
}

/* LED1 reports calibration without a computer. Completion owns both LEDs.
   No delays, interrupts, beeps or motor writes in this indicator path. */
static void key_indicator(uint32_t now)
{
    if (completion_signal) return;
    int on = 0;
    if (state != RUNNING && state != BRAKING) {
        if (yaw_phase == Y_READY) on = 1;
        else if (yaw_phase == Y_WARM || yaw_phase == Y_CAL)
            on = ((uint32_t)(now - yaw_warm_ms) / 250U) % 2U == 0U;
        else if (yaw_phase == Y_FAIL) on = (now / 100U) % 2U == 0U;
    }
    if (ready_led_on != (on != 0)) {
        ready_led_on = on != 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
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


void DualPITest_EnableHeading(int mpu_initialized)
{
    yaw_available = mpu_initialized != 0;
    yaw_phase = yaw_available ? Y_NONE : Y_FAIL;
    yaw_samples = yaw_io_us = 0U;
    yaw_angle = yaw_reference = yaw_error = yaw_diff = 0.0f;
    yaw_log_ms = HAL_GetTick();
}

static float wrap_degrees(float x)
{
    if (x > 180.0f) x -= 360.0f;
    if (x < -180.0f) x += 360.0f;
    return x;
}

static void begin_yaw_calibration(uint32_t now, int enabled)
{
    if (state == RUNNING || state == BRAKING || enabled || !yaw_available) return;
    drive(0, 0);
    clear_controllers();
    signal_set(0);
    yaw_phase = Y_WARM;
    yaw_warm_ms = now;
    yaw_samples = 0U;
    yaw_sum = 0;
    yaw_angle = yaw_reference = yaw_error = yaw_diff = 0.0f;
}

/* Called once per encoder sample in foreground. No blocking serial output. */
static void sample_yaw(uint32_t now, int enabled, int wheels_moved)
{
    if (yaw_phase == Y_WARM || yaw_phase == Y_CAL) {
        if (enabled || wheels_moved) { yaw_phase = Y_FAIL; return; }
        if (yaw_phase == Y_WARM) {
            if ((uint32_t)(now - yaw_warm_ms) < 2000U) return;
            yaw_phase = Y_CAL;
        }
    }
    if (yaw_phase != Y_CAL && yaw_phase != Y_READY) return;
    MPU6050_Raw raw;
    uint32_t before = DWT->CYCCNT;
    HAL_StatusTypeDef result = MPU6050_ReadRaw(&raw);
    uint32_t cycles = DWT->CYCCNT;
    yaw_io_us = (uint32_t)((uint64_t)(cycles - before) * 1000000U / hz);
    if (result != HAL_OK || yaw_io_us > 5000U ||
        raw.gz >= 32760 || raw.gz <= -32760) { yaw_phase = Y_FAIL; return; }
    if (yaw_phase == Y_CAL) {
        const int16_t g[3] = {raw.gx, raw.gy, raw.gz};
        for (unsigned i = 0; i < 3U; ++i) {
            if (yaw_samples == 0U || g[i] < yaw_min[i]) yaw_min[i] = g[i];
            if (yaw_samples == 0U || g[i] > yaw_max[i]) yaw_max[i] = g[i];
        }
        yaw_sum += raw.gz;
        if (++yaw_samples < 500U) return;
        if ((int32_t)yaw_max[0] - yaw_min[0] > 1000 ||
            (int32_t)yaw_max[1] - yaw_min[1] > 1000 ||
            (int32_t)yaw_max[2] - yaw_min[2] > 300) { yaw_phase = Y_FAIL; return; }
        yaw_bias = (float)yaw_sum / 500.0f;
        yaw_angle = yaw_reference = yaw_error = yaw_diff = 0.0f;
        yaw_previous_rate = ((float)raw.gz - yaw_bias) / 131.0f;
        yaw_cycles = cycles;
        yaw_phase = Y_READY;
        return;
    }
    uint32_t elapsed = cycles - yaw_cycles;
    yaw_cycles = cycles;
    if ((uint32_t)((uint64_t)elapsed * 1000000U / hz) < 5000U ||
        (uint32_t)((uint64_t)elapsed * 1000000U / hz) > 20000U) {
        yaw_phase = Y_FAIL; return;
    }
    float rate = ((float)raw.gz - yaw_bias) / 131.0f;
    yaw_angle = wrap_degrees(yaw_angle + 0.5f * (rate + yaw_previous_rate) * ((float)elapsed / hz));
    yaw_previous_rate = rate;
    yaw_error = wrap_degrees(yaw_reference - yaw_angle);
    yaw_diff = DPT_HEADING_GAIN * yaw_error;
    if (yaw_diff > DPT_HEADING_MAX_DIFF) yaw_diff = DPT_HEADING_MAX_DIFF;
    if (yaw_diff < -DPT_HEADING_MAX_DIFF) yaw_diff = -DPT_HEADING_MAX_DIFF;
}

static void request_motion(Motion requested, uint32_t now)
{
    if (state == BRAKING || yaw_phase == Y_WARM || yaw_phase == Y_CAL) return;
    if ((requested == MOVE_HEADING || is_distance(requested)) && (state == RUNNING || yaw_phase != Y_READY)) return;
    if (state == RUNNING && (requested == MOVE_TEST || requested == motion)) return;
    if (requested == MOVE_LINE) {
        line_decode(); /* Fresh GPIO gate; never start from an old OLED frame. */
        if (!line_center_ready || (uint32_t)(now - line_sample_ms) > 15U ||
            !line_valid || !line_black[1] || !line_black[2] || line_count != 2U) return;
    }
    if (!on_seen || (uint32_t)(now - on_ms) < 100U) stop(OFF, '-');
    else if (!adc_ok || (uint32_t)(now - adc_ms) > 150U) stop(ADC_BAD, '-');
    else if (bat_raw < DPT_MIN_BAT_RAW) stop(LOW, '-');
    else if (primed) {
        uint8_t was_running = state == RUNNING;
        drive(0, 0);
        clear_controllers();
        signal_set(0);
        motion = requested;
        float speed = (requested == MOVE_TEST || requested == MOVE_HEADING || is_distance(requested)) ? DPT_TARGET_MMPS : (float)manual_speed;
        targets[LEFT] = targets[RIGHT] = speed;
        if (requested == MOVE_BACK) targets[LEFT] = targets[RIGHT] = -speed;
        if (requested == MOVE_LEFT) targets[LEFT] = speed * 0.5f;
        if (requested == MOVE_RIGHT) targets[RIGHT] = speed * 0.5f;
        if (requested == MOVE_HEADING || is_distance(requested)) {
            yaw_reference = yaw_angle;
            yaw_error = yaw_diff = 0.0f;
        }
        run_duration = (requested == MOVE_TEST || requested == MOVE_HEADING || is_distance(requested)) ? DPT_DURATION_MS : manual_duration;
        distance_tracking = 0;
        if (is_distance(requested)) {
            for (unsigned i = 0; i < 2U; ++i) {
                distance_count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(wheels[i].encoder);
                distance_mm[i] = 0.0f;
            }
            distance_goal = requested == MOVE_ROUTE ? DPT_ROUTE_MM : (requested == MOVE_LINE ? DPT_LINE_MM : DPT_DISTANCE_MM);
            distance_cruise = requested == MOVE_ROUTE ? DPT_ROUTE_CRUISE_MMPS : (requested == MOVE_LINE ? DPT_LINE_CRUISE_MMPS : DPT_TARGET_MMPS);
            distance_slow = requested == MOVE_ROUTE ? DPT_ROUTE_SLOW_MM : DPT_DISTANCE_SLOW_MM;
            distance_base = requested == MOVE_ROUTE ? DPT_ROUTE_START_MMPS : (requested == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_TARGET_MMPS);
            line_invalid_seen = 0U;
            line_diff = 0.0f;
            targets[LEFT] = targets[RIGHT] = distance_base;
            distance_tracking = 1;
            settle_seen = 0;
            run_duration = requested == MOVE_ROUTE ? DPT_ROUTE_TIMEOUT_MS :
                (requested == MOVE_LINE ? DPT_LINE_TIMEOUT_MS : DPT_DISTANCE_TIMEOUT_MS);
        }
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
        if (strcmp(in->text, "CAL") == 0 && state != RUNNING) cal_requested = 1;
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
        if (state != RUNNING && in->text[0] == 'V' &&
            unsigned_value(in->text + 1, &value) &&
            value >= 40U && value <= DPT_MANUAL_MAX_MMPS)
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
    signal_set(0);
    signal_ms = 0U;
    yaw_available = cal_requested = 0;
    yaw_phase = Y_NONE;
    distance_tracking = settle_seen = 0;
    distance_mm[LEFT] = distance_mm[RIGHT] = 0.0f;
    distance_base = distance_cruise = DPT_TARGET_MMPS;
    distance_goal = DPT_DISTANCE_MM;
    distance_slow = DPT_DISTANCE_SLOW_MM;
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
    TaskOLED_Begin(hz); /* Boot reset delay occurs before priming the 10ms loop. */
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
    key_raw = key_stable = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
    key_press_enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    key_armed = 0; /* Even at boot, require 30ms stable release first. */
    key_change_ms = key_log_ms = now;
    key_events = 0U;
    key_off_pending = key_press_busy = 0U;
    key_hold_ms = now;
    key_action = KEY_NONE;
    oled_render_ms = now - 200U;
    oled_log_ms = line_log_ms = now;
    memset(line_raw, 0, sizeof line_raw);
    memset(line_black, 0, sizeof line_black);
    key_motion = MOVE_ROUTE; /* Boot remains the accepted independent 1m task. */
    line_valid = line_count = line_center_seen = line_center_ready = line_invalid_seen = 0U;
    line_center_ms = line_sample_ms = line_invalid_ms = now;
    line_error = line_diff = 0.0f;
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
    int limit = (motion == MOVE_TEST || motion == MOVE_HEADING || is_distance(motion)) ? DPT_MAX_CMD : DPT_MANUAL_MAX_CMD;
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

/* Electrical OLED order is O1..O4. Field-measured physical L->R is O4..O1;
   user corrected polarity and rechecked white=1111 / centre=1001. */
static void read_line(void)
{
    line_raw[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET;
    line_raw[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET;
    line_raw[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET;
    line_raw[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET;
}

/* Digital centroid in physical left->right order. Only contiguous 1..3 black
   probes are a line; all-white/all-black/split shapes are never an endpoint. */
static void line_decode(void)
{
    static const int weight[4] = {-3, -1, 1, 3};
    read_line();
    int sum = 0, first = -1, last = -1;
    line_count = 0U;
    for (unsigned i = 0; i < 4U; ++i) {
        line_black[i] = line_raw[3U - i] == DPT_LINE_BLACK_LEVEL;
        if (line_black[i]) {
            if (first < 0) first = (int)i;
            last = (int)i;
            ++line_count;
            sum += weight[i];
        }
    }
    line_valid = line_count > 0U && line_count < 4U && last - first + 1 == (int)line_count;
    if (line_valid) line_error = (float)sum / line_count;
}

static void line_sample(uint32_t now, int timing_ok)
{
    if (key_motion != MOVE_LINE && motion != MOVE_LINE) return;
    if (!timing_ok) { line_center_seen = line_center_ready = 0U; return; }
    line_decode();
    line_sample_ms = now;
    if (line_valid && line_count == 2U && line_black[1] && line_black[2]) {
        if (!line_center_seen) { line_center_seen = 1U; line_center_ms = now; }
        line_center_ready = (uint32_t)(now - line_center_ms) >= DPT_LINE_CENTER_MS;
    } else line_center_seen = line_center_ready = 0U;
}

static void line_targets(uint32_t now, float base)
{
    if (!line_valid) {
        /* Immediate zero command; only transient recovery <100ms may resume.
           A latched fault always needs a fresh centred start, never a search. */
        drive(0, 0);
        clear_controllers();
        targets[LEFT] = targets[RIGHT] = line_diff = 0.0f;
        if (!line_invalid_seen) { line_invalid_seen = 1U; line_invalid_ms = now; }
        else if ((uint32_t)(now - line_invalid_ms) >= DPT_LINE_INVALID_MS)
            stop(line_count == 0U ? LINE_LOST : LINE_BAD, '-');
        return;
    }
    line_invalid_seen = 0U;
    line_diff = DPT_LINE_GAIN * line_error * base / DPT_LINE_CRUISE_MMPS;
    float limit = DPT_LINE_MAX_DIFF * base / DPT_LINE_CRUISE_MMPS;
    if (line_diff > limit) line_diff = limit;
    if (line_diff < -limit) line_diff = -limit;
    /* Right-side black -> left wheel faster -> right turn. Both stay forward. */
    targets[LEFT] = base + line_diff;
    targets[RIGHT] = base - line_diff;
}

/* Low-rate text snapshots; SPI is sent in bounded chunks after control. */
static void update_oled(uint32_t now, int enabled)
{
    if ((uint32_t)(now - oled_render_ms) >= 200U) {
        static const char *const phases[] = {"NONE", "WARM", "CAL", "READY", "FAIL"};
        static const char *const states[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP", "YAW", "BRAKE", "TIMEOUT", "SETTLE", "LOST", "LINE"};
        const char *title = "TASK 1 A->B";
        if (motion == MOVE_LINE || (motion == MOVE_NONE && key_motion == MOVE_LINE)) title = "TEST LF20";
        else if (motion == MOVE_DISTANCE) title = "TEST GO20";
        else if (motion == MOVE_HEADING) title = "TEST GO";
        else if (motion == MOVE_TEST) title = "TEST RUN";
        else if (motion != MOVE_NONE && motion != MOVE_ROUTE) title = "MANUAL";
        char cal[24], run[24], progress[32];
        (void)snprintf(cal, sizeof cal, "YC %s EN %s", phases[yaw_phase], enabled ? "ON" : "OFF");
        (void)snprintf(run, sizeof run, "STATE %s", states[state]);
        if (!enabled) {
            read_line();
            (void)snprintf(progress, sizeof progress, "LF %u %u %u %u",
                (unsigned)line_raw[0], (unsigned)line_raw[1], (unsigned)line_raw[2], (unsigned)line_raw[3]);
        } else if (motion == MOVE_LINE || (motion == MOVE_NONE && key_motion == MOVE_LINE))
            (void)snprintf(progress, sizeof progress, "LF%u%u%u%u S%d/200",
                (unsigned)line_raw[0], (unsigned)line_raw[1], (unsigned)line_raw[2], (unsigned)line_raw[3],
                rounded((distance_mm[LEFT] + distance_mm[RIGHT]) * 0.5f));
        else if (is_distance(motion))
            (void)snprintf(progress, sizeof progress, "S %d/%dmm",
                rounded((distance_mm[LEFT] + distance_mm[RIGHT]) * 0.5f), rounded(distance_goal));
        else if (motion == MOVE_NONE)
            (void)snprintf(progress, sizeof progress, "S 0/1000mm");
        else (void)snprintf(progress, sizeof progress, "V %dmm/s", state == RUNNING ? rounded(targets[LEFT]) : 0);
        const char *lines[4] = {title, cal, run, progress};
        (void)TaskOLED_SetLines(lines);
        oled_render_ms = now; /* Retry at next snapshot if a frame is still busy. */
    }
    TaskOLED_Poll(now);
    if ((uint32_t)(now - oled_log_ms) >= 1000U) {
        char msg[80];
        int n = snprintf(msg, sizeof msg, "OLED TRY=SSD1306 SPI=H1 IO_US=%lu\r\n", (unsigned long)TaskOLED_LastIOUs());
        oled_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
    if ((!enabled || key_motion == MOVE_LINE || motion == MOVE_LINE) && (uint32_t)(now - line_log_ms) >= 1000U) {
        char msg[128];
        int n;
        if (key_motion == MOVE_LINE || motion == MOVE_LINE)
            n = snprintf(msg, sizeof msg, "LINE RAW=%u%u%u%u E100=%d C=%d VALID=%u CENTER=%u MODE=LF20\r\n",
                (unsigned)line_raw[0], (unsigned)line_raw[1], (unsigned)line_raw[2], (unsigned)line_raw[3],
                rounded(line_error * 100.0f), rounded(line_diff), (unsigned)line_valid, (unsigned)line_center_ready);
        else n = snprintf(msg, sizeof msg, "LINE O1=%u O2=%u O3=%u O4=%u\r\n",
            (unsigned)line_raw[0], (unsigned)line_raw[1], (unsigned)line_raw[2], (unsigned)line_raw[3]);
        line_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }

}

void DualPITest_Poll(uint32_t tick10_count)
{
    if (!active) return;
    uint32_t now = HAL_GetTick();
    int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    uint8_t was_busy = state == RUNNING || state == BRAKING || yaw_phase == Y_WARM || yaw_phase == Y_CAL;
    KeyEvent key_pressed = key_poll(now, enabled, was_busy);
    if (!enabled) {
        on_seen = 0;
        if (completion_signal) signal_set(0);
        if (state == RUNNING || state == BRAKING) stop(OFF, '-');
    } else if (!on_seen) { on_seen = 1; on_ms = now; }
    if (state == RUNNING && (uint32_t)(now - start_ms) >= run_duration)
        stop(is_distance(motion) ? TIMEOUT : DONE, '-');
    if (state == BRAKING && (uint32_t)(now - brake_ms) >= DPT_DISTANCE_BRAKE_TIMEOUT_MS)
        stop(SETTLE_BAD, '-');
    if (completion_signal && (uint32_t)(now - signal_ms) >= DPT_ROUTE_SIGNAL_MS) signal_set(0);
    if (switching && (uint32_t)(now - switch_ms) >= DPT_SWITCH_BRAKE_MS) switching = 0;

    if ((uint32_t)(now - adc_ms) >= 100U) {
        adc_ms = now;
        adc_ok = 0;
        if (HAL_ADC_Start(adc) == HAL_OK && HAL_ADC_PollForConversion(adc, 1U) == HAL_OK) {
            bat_raw = HAL_ADC_GetValue(adc);
            adc_ok = HAL_ADC_Stop(adc) == HAL_OK;
        } else (void)HAL_ADC_Stop(adc);
        if (state == RUNNING || state == BRAKING) {
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
    /* OFF selection is separate from starting. No pending movement is kept.
       Serial STOP/RX errors or a busy press cannot select or queue a restart. */
    if (!enabled && !was_busy && !stop_requested &&
        (requested == MOVE_LINE || requested == MOVE_ROUTE)) {
        select_key_motion(requested, now);
        requested = MOVE_NONE;
    }
    if (stop_requested) key_off_pending = 0U;
    if (key_pressed) {
        key_action = KEY_BLOCK;
        /* Both serial ports have been scanned: STOP/RX fault has priority. */
        if (!stop_requested) {
            if (was_busy || state == RUNNING || state == BRAKING || yaw_phase == Y_WARM || yaw_phase == Y_CAL) {
                stop(STOPPED, '-');
                stop_requested = 1;
                key_action = KEY_STOP;
            } else if (key_pressed == KEY_EVENT_HOLD && !enabled && !cal_requested) {
                select_key_motion(key_motion == MOVE_ROUTE ? MOVE_LINE : MOVE_ROUTE, now);
                requested = MOVE_NONE;
                key_action = KEY_MODE;
            } else if (key_pressed == KEY_EVENT_SHORT && !enabled && yaw_available) {
                cal_requested = 1;
                key_action = KEY_CAL;
            } else if (key_pressed == KEY_EVENT_SHORT && enabled && yaw_phase == Y_READY && !cal_requested) {
                requested = key_motion;
                key_action = KEY_START;
            } else key_action = KEY_WAIT;
        }
        key_log_ms = now - 1000U; /* Prompt short diagnostic, never blocking. */
    }
    if (cal_requested) {
        if (!stop_requested) begin_yaw_calibration(now, enabled);
        cal_requested = 0;
        requested = MOVE_NONE; /* CAL never queues movement for later. */
    }
    if (requested != MOVE_NONE && !stop_requested) request_motion(requested, now);
    if (key_pressed && key_action == KEY_START && (state != RUNNING || motion != key_motion))
        key_action = KEY_BLOCK;

    if (!primed || tick10_count != old_tick) {
        uint16_t count[2];
        for (unsigned i = 0; i < 2U; ++i)
            count[i] = (uint16_t)__HAL_TIM_GET_COUNTER(wheels[i].encoder);
        uint32_t cycles = DWT->CYCCNT;
        if (primed) {
            uint32_t elapsed = cycles - old_cycles;
            dt_us = (uint32_t)((uint64_t)elapsed * 1000000U / hz);
            if ((uint32_t)(now - old_ms) > 1000U || elapsed == 0U) {
                line_sample(now, 0);
                if (yaw_phase != Y_NONE) yaw_phase = Y_FAIL;
                if (state == RUNNING || state == BRAKING) stop(GAP, '-');
                for (unsigned i = 0; i < 2U; ++i) wheels[i].velocity = 0.0f;
            } else {
                float seconds = (float)elapsed / hz;
                for (unsigned i = 0; i < 2U; ++i)
                    wheels[i].velocity = sign[i] * (float)delta16(count[i], wheels[i].old_count)
                        * DPT_CIRC_M * 1000.0f / cpr[i] / seconds;
                int wheels_moved = count[LEFT] != wheels[LEFT].old_count || count[RIGHT] != wheels[RIGHT].old_count;
                if (tick10_count - old_tick != 1U || dt_us < 5000U || dt_us > 15000U) {
                    if (yaw_phase == Y_CAL || yaw_phase == Y_READY) yaw_phase = Y_FAIL;
                    if (state == RUNNING || state == BRAKING) stop(GAP, '-');
                } else sample_yaw(now, enabled, wheels_moved);
                line_sample(now, tick10_count - old_tick == 1U && dt_us >= 5000U && dt_us <= 15000U);
                /* Sensor I/O is bounded; recheck physical enable before any drive commit. */
                if ((state == RUNNING || state == BRAKING) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET)
                    stop(OFF, '-');
                if ((state == RUNNING || state == BRAKING) &&
                    (motion == MOVE_HEADING || is_distance(motion)) &&
                    (yaw_phase != Y_READY || (motion != MOVE_LINE && (yaw_error > DPT_HEADING_MAX_ERROR || yaw_error < -DPT_HEADING_MAX_ERROR))))
                    stop(YAW_BAD, '-');
                /* Accumulate signed wheel travel only from the fresh start snapshot.
                   Reject timing/implausible counts before declaring distance completion. */
                if (distance_tracking && (state == RUNNING || state == BRAKING)) {
                    if (wheels[LEFT].velocity > 350.0f || wheels[LEFT].velocity < -350.0f)
                        stop(FAST, 'L');
                    else if (wheels[RIGHT].velocity > 350.0f || wheels[RIGHT].velocity < -350.0f)
                        stop(FAST, 'R');
                    else {
                        for (unsigned i = 0; i < 2U; ++i) {
                            distance_mm[i] += sign[i] * (float)delta16(count[i], distance_count[i])
                                * DPT_CIRC_M * 1000.0f / cpr[i];
                            distance_count[i] = count[i];
                        }
                        if (state == RUNNING && (motion != MOVE_LINE || line_valid) && (distance_mm[LEFT] + distance_mm[RIGHT]) * 0.5f >= distance_goal)
                            stop(BRAKING, '-');
                        if (state == BRAKING) {
                            if (wheels[LEFT].velocity >= -5.0f && wheels[LEFT].velocity <= 5.0f &&
                                wheels[RIGHT].velocity >= -5.0f && wheels[RIGHT].velocity <= 5.0f) {
                                if (!settle_seen) { settle_seen = 1; settle_ms = now; }
                                else if ((uint32_t)(now - settle_ms) >= DPT_DISTANCE_SETTLE_MS) stop(DONE, '-');
                            } else settle_seen = 0;
                        }
                    }
                }
                if (state == RUNNING && (motion == MOVE_HEADING || is_distance(motion))) {
                    if (yaw_phase != Y_READY || (motion != MOVE_LINE && (yaw_error > DPT_HEADING_MAX_ERROR || yaw_error < -DPT_HEADING_MAX_ERROR)))
                        stop(YAW_BAD, '-');
                    else {
                        float base = DPT_TARGET_MMPS;
                        if (is_distance(motion)) {
                            base = distance_cruise;
                            float remaining = distance_goal - (distance_mm[LEFT] + distance_mm[RIGHT]) * 0.5f;
                            if (remaining < distance_slow) {
                                base = distance_cruise * remaining / distance_slow;
                                if (base < DPT_DISTANCE_MIN_MMPS) base = DPT_DISTANCE_MIN_MMPS;
                            }
                            if (motion == MOVE_ROUTE || motion == MOVE_LINE) {
                                float ramp = (motion == MOVE_LINE ? DPT_LINE_START_MMPS : DPT_ROUTE_START_MMPS) + DPT_ROUTE_ACCEL_MMPS2 * ((float)(uint32_t)(now - start_ms) / 1000.0f);
                                if (base > ramp) base = ramp;
                            }
                            distance_base = base;
                            float max_diff = DPT_HEADING_MAX_DIFF * base / distance_cruise;
                            if (yaw_diff > max_diff) yaw_diff = max_diff;
                            if (yaw_diff < -max_diff) yaw_diff = -max_diff;
                        }
                        if (motion == MOVE_LINE) line_targets(now, base);
                        else {
                            targets[LEFT] = base - yaw_diff;
                            targets[RIGHT] = base + yaw_diff;
                        }
                    }
                }
                if (state == RUNNING) {
                    if (tick10_count - old_tick != 1U || dt_us < 5000U || dt_us > 15000U)
                        stop(GAP, '-');
                    else if (!switching && (motion != MOVE_LINE || line_valid)) {
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

    if ((uint32_t)(now - log_ms) >= (is_distance(motion) ? 400U : 200U)) {
        static const char *const names[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP", "YAW", "BRAKE", "TIMEOUT", "SETTLE", "LOST", "LINE"};
        static const char *const modes[] = {"-", "TEST", "FWD", "BACK", "LEFT", "RIGHT", "HEAD", "DIST", "DIST", "LINE"};
        char msg[224];
        int n = snprintf(msg, sizeof msg,
            "DPI T=%d L=%d R=%d UL=%d UR=%d IL=%d IR=%d BAT=%lu DT=%lu ST=%s F=%c RXE=%lu TXD=%lu M=%s TL=%d TR=%d V=%u SEC=%lu\r\n",
            state == RUNNING ? (is_distance(motion) ? rounded(distance_base) : ((motion == MOVE_TEST || motion == MOVE_HEADING || is_distance(motion)) ? rounded(DPT_TARGET_MMPS) : (int)manual_speed)) : 0,
            rounded(wheels[LEFT].velocity), rounded(wheels[RIGHT].velocity),
            wheels[LEFT].command, wheels[RIGHT].command,
            rounded(wheels[LEFT].integral), rounded(wheels[RIGHT].integral),
            (unsigned long)bat_raw, (unsigned long)dt_us, names[state], fault_wheel,
            (unsigned long)(LabUART_RxErrors() + (bluetooth_on ? LabUART_RxErrorsPort(LAB_UART_BT) : 0U)),
            (unsigned long)(LabUART_TxDropped() + (bluetooth_on ? LabUART_TxDroppedPort(LAB_UART_BT) : 0U)),
            modes[motion], state == RUNNING ? rounded(targets[LEFT]) : 0,
            state == RUNNING ? rounded(targets[RIGHT]) : 0, manual_speed,
            (unsigned long)(manual_duration / 1000U));
        if (is_distance(motion) && n > 1 && n < (int)sizeof msg) {
            n -= 2; /* replace trailing CRLF with distance fields */
            int added = snprintf(msg + n, sizeof msg - (size_t)n,
                " S=%d SL=%d SR=%d G=%d\r\n",
                rounded((distance_mm[LEFT] + distance_mm[RIGHT]) * 0.5f),
                rounded(distance_mm[LEFT]), rounded(distance_mm[RIGHT]), rounded(distance_goal));
            if (added <= 0 || added >= (int)(sizeof msg - (size_t)n)) n = (int)sizeof msg;
            else n += added;
        }
        if ((motion == MOVE_ROUTE || motion == MOVE_LINE) && n > 1 && n < (int)sizeof msg) {
            n -= 2;
            int added = snprintf(msg + n, sizeof msg - (size_t)n, " SIG=%u\r\n", (unsigned)completion_signal);
            if (added <= 0 || added >= (int)(sizeof msg - (size_t)n)) n = (int)sizeof msg;
            else n += added;
        }
        log_ms = now;
        if (n <= 0 || n >= (int)sizeof msg) stop(UART_BAD, '-');
        else {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
        /* A full TX queue drops this log frame; control/RX keep running. */
    }

    if (yaw_available && (uint32_t)(now - yaw_log_ms) >= 1000U) {
        static const char *const phases[] = {"NONE", "WARM", "CAL", "READY", "FAIL"};
        char msg[128];
        int n = snprintf(msg, sizeof msg,
            "HDG YC=%s N=%lu BZ100=%d Y100=%d E100=%d C=%d IO_US=%lu\r\n",
            phases[yaw_phase], (unsigned long)yaw_samples, rounded(yaw_bias * 100.0f),
            rounded(yaw_angle * 100.0f), rounded(yaw_error * 100.0f), rounded(yaw_diff),
            (unsigned long)yaw_io_us);
        yaw_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }
    key_indicator(now);
    if ((uint32_t)(now - key_log_ms) >= 1000U) {
        static const char *const actions[] = {"-", "CAL", "START", "STOP", "WAIT", "BLOCK", "MODE"};
        char msg[96];
        int n = snprintf(msg, sizeof msg, "KEY EN=%u RAW=%u ARM=%u EVT=%lu ACT=%s %s\r\n",
            (unsigned)enabled, (unsigned)key_raw, (unsigned)key_armed,
            (unsigned long)key_events, actions[key_action], key_motion == MOVE_LINE ? "TEST=LF20" : "TASK=1");
        key_log_ms = now;
        if (n > 0 && n < (int)sizeof msg) {
            (void)LabUART_TryWrite(msg, (uint16_t)n);
            if (bluetooth_on) (void)LabUART_TryWritePort(LAB_UART_BT, msg, (uint16_t)n);
        }
    }

    update_oled(now, enabled);

}
