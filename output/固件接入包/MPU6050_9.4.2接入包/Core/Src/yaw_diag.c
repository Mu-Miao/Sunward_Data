#include "yaw_diag.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

enum { IDLE, WARMUP, CALIBRATING, TRACKING, FAULT };
static unsigned phase;
static UART_HandleTypeDef *port;
static uint32_t warmup_ms, sample_ms, sample_cycles, zero_ms, log_ms;
static uint32_t clock_hz, cal_count;
static int32_t sum_z;
static int16_t min_g[3], max_g[3];
static float bias_z, yaw_deg, previous_rate;

static void send_text(const char *text)
{
    size_t n = strlen(text);
    if (port == NULL || HAL_UART_Transmit(port, (uint8_t *)text,
            (uint16_t)n, 50U) != HAL_OK) phase = FAULT;
}
static void fail(const char *text)
{
    phase = FAULT;
    send_text(text);
}
/* Avoid floating-point printf requirements in newlib-nano. */
static void fixed2(char *text, size_t size, float value)
{
    int32_t scaled = (int32_t)(value * 100.0f +
                              (value >= 0.0f ? 0.5f : -0.5f));
    long magnitude = scaled < 0 ? -(long)scaled : (long)scaled;
    (void)snprintf(text, size, "%s%ld.%02ld", scaled < 0 ? "-" : "",
                   magnitude / 100L, magnitude % 100L);
}
void YawDiag_Begin(UART_HandleTypeDef *uart)
{
    port = uart;
    clock_hz = HAL_RCC_GetHCLKFreq();
    phase = IDLE;
    if (uart == NULL || clock_hz < 1000000U) {
        fail("YAW INIT FAIL\r\n");
        return;
    }
    /* SoftI2C_Init already uses DWT; verify it is still running. */
    uint32_t before = DWT->CYCCNT;
    for (unsigned i = 0U; i < 64U; ++i) { __NOP(); }
    if (DWT->CYCCNT == before) {
        fail("YAW CLOCK FAIL\r\n");
        return;
    }
    cal_count = 0U;
    sum_z = 0;
    bias_z = yaw_deg = previous_rate = 0.0f;
    warmup_ms = HAL_GetTick();
    phase = WARMUP;
    send_text("YAW WARMUP 2s; keep car still until CAL OK\r\n");
}
void YawDiag_Poll(void)
{
    uint32_t now_ms = HAL_GetTick();
    if (phase == WARMUP) {
        if ((uint32_t)(now_ms - warmup_ms) < 2000U) return;
        phase = CALIBRATING;
        sample_ms = now_ms;
        sample_cycles = DWT->CYCCNT;
        send_text("YAW CAL: keep still, 500 samples (about 5s)\r\n");
        return;
    }
    if (phase != CALIBRATING && phase != TRACKING) return;
    if ((uint32_t)(now_ms - sample_ms) < 10U) return;
    sample_ms = now_ms;
    MPU6050_Raw raw;
    if (MPU6050_ReadRaw(&raw) != HAL_OK) {
        fail("YAW READ FAIL; reset to retry\r\n");
        return;
    }
    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t elapsed_cycles = now_cycles - sample_cycles;
    sample_cycles = now_cycles;
    uint32_t dt_us = elapsed_cycles / (clock_hz / 1000000U);
    if (elapsed_cycles > clock_hz / 1000U * 30U) {
        fail("YAW GAP >30ms; angle invalid, reset to retry\r\n");
        return;
    }
    if (phase == CALIBRATING) {
        const int16_t g[3] = {raw.gx, raw.gy, raw.gz};
        for (unsigned i = 0U; i < 3U; ++i) {
            if (cal_count == 0U || g[i] < min_g[i]) min_g[i] = g[i];
            if (cal_count == 0U || g[i] > max_g[i]) max_g[i] = g[i];
        }
        sum_z += raw.gz;
        ++cal_count;
        if (cal_count < 500U) return;
        /* This rejects obvious movement; constant slow rotation can pass.
           The operator must keep the car completely stationary. */
        if ((int32_t)max_g[0] - min_g[0] > 1000 ||
            (int32_t)max_g[1] - min_g[1] > 1000 ||
            (int32_t)max_g[2] - min_g[2] > 300) {
            fail("YAW CAL UNSTABLE; keep still and reset to retry\r\n");
            return;
        }
        bias_z = (float)sum_z / 500.0f;
        previous_rate = ((float)raw.gz - bias_z) / 131.0f;
        yaw_deg = 0.0f;
        zero_ms = HAL_GetTick();
        log_ms = zero_ms;
        char bias[24], msg[112];
        fixed2(bias, sizeof bias, bias_z);
        int n = snprintf(msg, sizeof msg,
            "YAW CAL OK BIAS_Z=%s SPAN_Z=%ld; YAW=0.00 AGE_MS=0\r\n",
            bias, (long)((int32_t)max_g[2] - min_g[2]));
        phase = TRACKING;
        if (n > 0 && n < (int)sizeof msg) send_text(msg);
        sample_cycles = DWT->CYCCNT; /* zero time is after CAL OK output */
        sample_ms = HAL_GetTick();
        zero_ms = sample_ms;
        log_ms = zero_ms;
        return;
    }
    /* Configured +/-250 dps: 131 raw per degree/s. Left turn is positive. */
    float rate = ((float)raw.gz - bias_z) / 131.0f;
    float dt = (float)elapsed_cycles / (float)clock_hz;
    if (raw.gz >= 32760 || raw.gz <= -32760) {
        fail("YAW SATURATED; angle invalid, reset to retry\r\n");
        return;
    }
    yaw_deg += 0.5f * (previous_rate + rate) * dt;
    previous_rate = rate;
    /* Bound conversion to integer display even if left running for hours. */
    if (yaw_deg > 1000000.0f || yaw_deg < -1000000.0f) {
        fail("YAW OUT OF RANGE; reset to retry\r\n");
        return;
    }
    if ((uint32_t)(HAL_GetTick() - log_ms) >= 200U) {
        log_ms = HAL_GetTick();
        char angle[24], omega[24], msg[120];
        fixed2(angle, sizeof angle, yaw_deg);
        fixed2(omega, sizeof omega, rate);
        int n = snprintf(msg, sizeof msg,
            "YAW=%s W=%s GZ=%d DT_US=%lu AGE_MS=%lu\r\n",
            angle, omega, (int)raw.gz, (unsigned long)dt_us,
            (unsigned long)(log_ms - zero_ms));
        if (n > 0 && n < (int)sizeof msg) send_text(msg);
    }
}
