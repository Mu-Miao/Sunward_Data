#include "wheel_speed_diag.h"
#include <stdio.h>

static UART_HandleTypeDef *diag_uart;
static TIM_HandleTypeDef *diag_left, *diag_right;
static uint32_t clock_hz, prev_cycles, prev_ms, prev_tick;
static uint16_t prev_left, prev_right;
static uint8_t active, primed;
static int32_t sum_left, sum_right;
static uint64_t sum_cycles;
static uint32_t samples, missed, min_us, max_us;

static int32_t delta16(uint16_t now, uint16_t before)
{
    uint32_t d = ((uint32_t)now - (uint32_t)before) & 0xffffU;
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

static void reset_window(void)
{
    sum_left = sum_right = 0;
    sum_cycles = 0;
    samples = missed = max_us = 0;
    min_us = UINT32_MAX;
}

static int32_t rounded_mmps(float speed)
{
    float mmps = speed * 1000.0f;
    return (int32_t)(mmps >= 0.0f ? mmps + 0.5f : mmps - 0.5f);
}

HAL_StatusTypeDef WheelSpeedDiag_Begin(UART_HandleTypeDef *uart,
                                     TIM_HandleTypeDef *left,
                                     TIM_HandleTypeDef *right)
{
    active = primed = 0;
    if (uart == NULL || left == NULL || right == NULL ||
        WHEEL_DIAG_LEFT_CPR <= 0.0f || WHEEL_DIAG_RIGHT_CPR <= 0.0f ||
        WHEEL_DIAG_CIRC_M <= 0.0f) return HAL_ERROR;

    clock_hz = HAL_RCC_GetHCLKFreq();
    if (clock_hz < 1000000U) return HAL_ERROR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    HAL_Delay(1U);
    if (DWT->CYCCNT == before) return HAL_ERROR;

    diag_uart = uart;
    diag_left = left;
    diag_right = right;
    reset_window();
    static const char banner[] =
        "SPD DIAG: hand turn; L=TIM8 R=-TIM4; no motor commands\r\n";
    if (HAL_UART_Transmit(uart, (uint8_t *)banner,
                          sizeof banner - 1U, 20U) != HAL_OK) return HAL_ERROR;
    active = 1;
    return HAL_OK;
}

void WheelSpeedDiag_Poll(uint32_t tick10_count)
{
    if (!active) return;
    if (primed && tick10_count == prev_tick) return;

    uint16_t now_left = (uint16_t)__HAL_TIM_GET_COUNTER(diag_left);
    uint16_t now_right = (uint16_t)__HAL_TIM_GET_COUNTER(diag_right);
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t now_ms = HAL_GetTick();
    if (!primed) {
        primed = 1;
    } else if ((uint32_t)(now_ms - prev_ms) > 1000U) {
        /* Long pauses cannot be interpreted as a normal 10 ms sample. */
        reset_window();
        static const char msg[] = "SPD GAP: measurement window restarted\r\n";
        if (HAL_UART_Transmit(diag_uart, (uint8_t *)msg,
                              sizeof msg - 1U, 20U) != HAL_OK) active = 0;
    } else {
        uint32_t cycles = now_cycles - prev_cycles;
        if (cycles == 0U) { active = 0; return; }
        uint32_t dt_us = (uint32_t)((uint64_t)cycles * 1000000U / clock_hz);
        sum_left += delta16(now_left, prev_left);
        sum_right -= delta16(now_right, prev_right);
        sum_cycles += cycles;
        ++samples;
        uint32_t released = tick10_count - prev_tick;
        if (released > 1U) missed += released - 1U;
        if (dt_us < min_us) min_us = dt_us;
        if (dt_us > max_us) max_us = dt_us;
    }
    prev_left = now_left;
    prev_right = now_right;
    prev_cycles = now_cycles;
    prev_ms = now_ms;
    prev_tick = tick10_count;

    if (sum_cycles >= (uint64_t)clock_hz / 2U) {
        float dt_s = (float)sum_cycles / (float)clock_hz;
        float left_mps = (float)sum_left * WHEEL_DIAG_CIRC_M /
                         WHEEL_DIAG_LEFT_CPR / dt_s;
        float right_mps = (float)sum_right * WHEEL_DIAG_CIRC_M /
                          WHEEL_DIAG_RIGHT_CPR / dt_s;
        char msg[128];
        int n = snprintf(msg, sizeof msg,
            "SPD L=%ld R=%ld mm/s N=%lu DT=%lu..%lu us MISS=%lu\r\n",
            (long)rounded_mmps(left_mps), (long)rounded_mmps(right_mps),
            (unsigned long)samples, (unsigned long)min_us,
            (unsigned long)max_us, (unsigned long)missed);
        reset_window();
        if (n <= 0 || n >= (int)sizeof msg ||
            HAL_UART_Transmit(diag_uart, (uint8_t *)msg, (uint16_t)n, 20U)
                != HAL_OK) active = 0;
    }
}
