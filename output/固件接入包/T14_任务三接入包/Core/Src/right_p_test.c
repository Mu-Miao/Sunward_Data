#include "right_p_test.h"
#include <stdio.h>
#include <string.h>

typedef enum { IDLE, RUNNING, DONE, OFF, LOW, ADC_BAD, GAP,
               REVERSE, FAST, STALL, UART_BAD, STOPPED } TestState;
static UART_HandleTypeDef *uart;
static TIM_HandleTypeDef *enc;
static ADC_HandleTypeDef *adc;
static void (*drive)(int, int);
static TestState state;
static uint8_t active, primed, adc_ok, on_seen, discard;
static uint32_t hz, old_cycles, old_tick, old_ms, start_ms;
static uint32_t adc_ms, log_ms, on_ms, bad_sign_ms, stall_ms;
static uint32_t bat_raw, dt_us;
static uint16_t old_count;
static float velocity;
static float integral;
static int command;
static char input[12];
static unsigned input_len;

static void stop(TestState why)
{
    drive(0, 0);
    command = 0;
    integral = 0.0f;
    state = why;
    bad_sign_ms = stall_ms = 0U;
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

static void accept_command(uint32_t now)
{
    input[input_len] = '\0';
    if (!discard && strcmp(input, "STOP") == 0) stop(STOPPED);
    if (!discard && strcmp(input, "RUN") == 0 && state != RUNNING) {
        if (!on_seen || (uint32_t)(now - on_ms) < 100U) stop(OFF);
        else if (!adc_ok || (uint32_t)(now - adc_ms) > 150U) stop(ADC_BAD);
        else if (bat_raw < RPT_MIN_BAT_RAW) stop(LOW);
        else if (primed) {
            start_ms = now;
            integral = 0.0f;
            bad_sign_ms = stall_ms = 0U;
            state = RUNNING;
            /* The first command is applied only on a new measurement tick. */
        }
    }
    input_len = 0;
    discard = 0;
}

HAL_StatusTypeDef RightPTest_Begin(UART_HandleTypeDef *u,
    TIM_HandleTypeDef *right_encoder, ADC_HandleTypeDef *battery_adc,
    void (*motor_ab)(int a, int b))
{
    active = 0;
    if (motor_ab == NULL) return HAL_ERROR;
    motor_ab(0, 0);
    if (u == NULL || right_encoder == NULL || battery_adc == NULL ||
        RPT_KP <= 0.0f || RPT_KI < 0.0f || RPT_KI > 100.0f ||
        RPT_MAX_CMD <= 0 || RPT_MAX_CMD > 1800 ||
        RPT_TARGET_MMPS <= 0.0f || RPT_TARGET_MMPS > 100.0f ||
        RPT_RIGHT_CPR <= 0.0f || RPT_CIRC_M <= 0.0f ||
        RPT_DURATION_MS == 0U || RPT_DURATION_MS > 8000U) return HAL_ERROR;
    hz = HAL_RCC_GetHCLKFreq();
    if (hz < 1000000U) return HAL_ERROR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    if (DWT->CYCCNT == before) return HAL_ERROR;
    uart = u; enc = right_encoder; adc = battery_adc; drive = motor_ab;
    state = IDLE;
    primed = adc_ok = on_seen = discard = 0;
    input_len = 0;
    command = 0; velocity = 0.0f; bat_raw = dt_us = 0;
    integral = 0.0f;
    bad_sign_ms = stall_ms = 0U;
    uint32_t now = HAL_GetTick();
    adc_ms = now - 100U;
    log_ms = now;
    static const char banner[] = "RPI: send RUN! or STOP!; right A only; 3s; PI active\r\n";
    if (HAL_UART_Transmit(u, (uint8_t *)banner,
                          sizeof banner - 1U, 20U) != HAL_OK) return HAL_ERROR;
    active = 1;
    return HAL_OK;
}

void RightPTest_Poll(uint32_t tick10_count)
{
    if (!active) return;
    uint32_t now = HAL_GetTick();
    int enabled = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET;
    if (!enabled) {
        on_seen = 0;
        if (state == RUNNING) stop(OFF);
    } else if (!on_seen) { on_seen = 1; on_ms = now; }
    if (state == RUNNING && (uint32_t)(now - start_ms) >= RPT_DURATION_MS)
        stop(DONE);

    /* ADC waits at most 1ms. This runs outside the TIM2 interrupt. */
    if ((uint32_t)(now - adc_ms) >= 100U) {
        adc_ms = now;
        adc_ok = 0;
        if (HAL_ADC_Start(adc) == HAL_OK &&
            HAL_ADC_PollForConversion(adc, 1U) == HAL_OK) {
            bat_raw = HAL_ADC_GetValue(adc);
            adc_ok = HAL_ADC_Stop(adc) == HAL_OK;
        } else (void)HAL_ADC_Stop(adc);
        if (state == RUNNING) {
            if (!adc_ok) stop(ADC_BAD);
            else if (bat_raw < RPT_MIN_BAT_RAW) stop(LOW);
        }
    }

    /* Complete commands only: RUN! / STOP!, or a newline terminator. */
    for (unsigned i = 0; i < 16U && __HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE); ++i) {
        uint8_t ch;
        if (HAL_UART_Receive(uart, &ch, 1U, 0U) != HAL_OK) {
            stop(UART_BAD); active = 0; return;
        }
        if (ch == '!' || ch == '\r' || ch == '\n') accept_command(now);
        else if (!discard && input_len < sizeof input - 1U) input[input_len++] = (char)ch;
        else discard = 1;
    }

    if (!primed || tick10_count != old_tick) {
        uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(enc);
        uint32_t cycles = DWT->CYCCNT;
        if (primed) {
            uint32_t elapsed = cycles - old_cycles;
            dt_us = (uint32_t)((uint64_t)elapsed * 1000000U / hz);
            if ((uint32_t)(now - old_ms) > 1000U || elapsed == 0U) {
                if (state == RUNNING) stop(GAP);
                velocity = 0.0f;
            } else {
                velocity = -(float)delta16(count, old_count) * RPT_CIRC_M *
                           1000.0f / RPT_RIGHT_CPR / ((float)elapsed / hz);
            }
            if (state == RUNNING) {
                if (tick10_count - old_tick != 1U || dt_us < 5000U || dt_us > 15000U)
                    stop(GAP);
                else if (velocity > 350.0f || velocity < -350.0f) stop(FAST);
                else {
                    if (velocity < -5.0f) bad_sign_ms += dt_us / 1000U;
                    else bad_sign_ms = 0;
                    if (bad_sign_ms >= 50U) stop(REVERSE);
                    else {
                        float error = RPT_TARGET_MMPS - velocity;
                        float candidate = integral + RPT_KI * error *
                                          ((float)elapsed / hz);
                        float trial = RPT_KP * error + candidate;
                        /* Conditional integration: do not wind up at either limit. */
                        if (!((trial > RPT_MAX_CMD && error > 0.0f) ||
                              (trial < 0.0f && error < 0.0f))) {
                            if (candidate < 0.0f) candidate = 0.0f;
                            if (candidate > RPT_MAX_CMD) candidate = RPT_MAX_CMD;
                            integral = candidate;
                        }
                        float out = RPT_KP * error + integral;
                        if (out < 0.0f) out = 0.0f;
                        if (out > RPT_MAX_CMD) out = RPT_MAX_CMD;
                        command = rounded(out);
                        if (command >= 1200 && velocity < 5.0f) stall_ms += dt_us / 1000U;
                        else stall_ms = 0;
                        if (stall_ms >= 500U) stop(STALL);
                        else drive(command, 0); /* A=right forward; B=left brake. */
                    }
                }
            }
        } else primed = 1;
        old_count = count; old_cycles = cycles; old_tick = tick10_count; old_ms = now;
    }

    if ((uint32_t)(now - log_ms) >= 200U) {
        static const char *const names[] = {"IDLE", "RUN", "DONE", "OFF", "LOW",
            "ADC", "GAP", "REV", "FAST", "STALL", "UART", "STOP"};
        char msg[100];
        int n = snprintf(msg, sizeof msg,
            "RPI T=%d V=%d U=%d I=%d BAT=%lu DT=%lu ST=%s\r\n",
            state == RUNNING ? rounded(RPT_TARGET_MMPS) : 0, rounded(velocity), command,
            rounded(integral), (unsigned long)bat_raw, (unsigned long)dt_us, names[state]);
        log_ms = now;
        if (n <= 0 || n >= (int)sizeof msg ||
            HAL_UART_Transmit(uart, (uint8_t *)msg, (uint16_t)n, 20U) != HAL_OK) {
            stop(UART_BAD); active = 0;
        }
    }
}
