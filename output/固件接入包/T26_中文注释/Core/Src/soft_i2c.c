#include "soft_i2c.h"

#define SCL GPIO_PIN_14
#define SDA GPIO_PIN_15
static uint32_t cycles_per_us;
static uint8_t initialized;

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { __NOP(); }
}
static void scl_low(void) { HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_RESET); }
static void sda_low(void) { HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_RESET); }
/* 开漏模式下置 SET 是释放总线，由上拉电阻拉高，不是主动输出高电平。 */
static void sda_release(void) { HAL_GPIO_WritePin(GPIOB, SDA, GPIO_PIN_SET); }
static HAL_StatusTypeDef scl_high(void)
{
    uint32_t start = DWT->CYCCNT;
    HAL_GPIO_WritePin(GPIOB, SCL, GPIO_PIN_SET);
    while (HAL_GPIO_ReadPin(GPIOB, SCL) == GPIO_PIN_RESET) {
        if ((uint32_t)(DWT->CYCCNT - start) >= 1000U * cycles_per_us)
            return HAL_TIMEOUT;
    }
    delay_us(5U);
    return HAL_OK;
}
uint8_t SoftI2C_Lines(void)
{
    uint8_t result = 0U;
    if (HAL_GPIO_ReadPin(GPIOB, SCL) == GPIO_PIN_SET) result |= 1U;
    if (HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_SET) result |= 2U;
    return result;
}
static HAL_StatusTypeDef start(void)
{
    sda_release();
    delay_us(5U);
    HAL_StatusTypeDef status = scl_high();
    if (status != HAL_OK) return status;
    if (HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET) return HAL_BUSY;
    sda_low();
    delay_us(5U);
    scl_low();
    delay_us(5U);
    return HAL_OK;
}
static HAL_StatusTypeDef stop(void)
{
    scl_low();
    sda_low();
    delay_us(5U);
    HAL_StatusTypeDef status = scl_high();
    sda_release();
    delay_us(5U);
    return status;
}
static HAL_StatusTypeDef write_byte(uint8_t value)
{
    for (unsigned bit = 0U; bit < 8U; ++bit) {
        if ((value & 0x80U) != 0U) sda_release(); else sda_low();
        delay_us(5U);
        HAL_StatusTypeDef status = scl_high();
        if (status != HAL_OK) return status;
        scl_low();
        delay_us(5U);
        value = (uint8_t)(value << 1);
    }
    sda_release();
    delay_us(5U);
    HAL_StatusTypeDef status = scl_high();
    if (status != HAL_OK) return status;
    uint8_t ack = HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET;
    scl_low();
    delay_us(5U);
    return ack ? HAL_OK : HAL_ERROR;
}
static HAL_StatusTypeDef read_byte(uint8_t *value, uint8_t acknowledge)
{
    uint8_t result = 0U;
    sda_release();
    for (unsigned bit = 0U; bit < 8U; ++bit) {
        delay_us(5U);
        HAL_StatusTypeDef status = scl_high();
        if (status != HAL_OK) return status;
        result = (uint8_t)(result << 1);
        if (HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_SET) result |= 1U;
        scl_low();
        delay_us(5U);
    }
    if (acknowledge) sda_low(); else sda_release();
    delay_us(5U);
    HAL_StatusTypeDef status = scl_high();
    scl_low();
    sda_release();
    delay_us(5U);
    if (status == HAL_OK) *value = result;
    return status;
}
static HAL_StatusTypeDef finish(HAL_StatusTypeDef status)
{
    HAL_StatusTypeDef stopped = stop();
    return status == HAL_OK ? stopped : status;
}
HAL_StatusTypeDef SoftI2C_Init(void)
{
    initialized = 0U;
    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (cycles_per_us == 0U) return HAL_ERROR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = DWT->CYCCNT;
    for (unsigned i = 0U; i < 64U; ++i) { __NOP(); }
    if (DWT->CYCCNT == before) return HAL_ERROR;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, SCL | SDA, GPIO_PIN_SET);
    GPIO_InitTypeDef pins = {0};
    pins.Pin = SCL | SDA;
    pins.Mode = GPIO_MODE_OUTPUT_OD;
    pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &pins);
    HAL_StatusTypeDef status = scl_high();
    if (status != HAL_OK) return status;
    /* 恢复 MCU 复位前可能停在字节传输中途的从设备。 */
    for (unsigned i = 0U; i < 9U &&
         HAL_GPIO_ReadPin(GPIOB, SDA) == GPIO_PIN_RESET; ++i) {
        scl_low();
        delay_us(5U);
        status = scl_high();
        if (status != HAL_OK) return status;
    }
    status = stop();
    if (status != HAL_OK) return status;
    if (SoftI2C_Lines() != 3U) return HAL_BUSY;
    initialized = 1U;
    return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    if (!initialized || addr > 0x7FU) return HAL_ERROR;
    HAL_StatusTypeDef status = start();
    if (status == HAL_OK) status = write_byte((uint8_t)(addr << 1));
    if (status == HAL_OK) status = write_byte(reg);
    if (status == HAL_OK) status = write_byte(value);
    return finish(status);
}
HAL_StatusTypeDef SoftI2C_ReadReg(uint8_t addr, uint8_t reg,
                              uint8_t *data, uint16_t size)
{
    if (!initialized || addr > 0x7FU || data == NULL || size == 0U)
        return HAL_ERROR;
    HAL_StatusTypeDef status = start();
    if (status == HAL_OK) status = write_byte((uint8_t)(addr << 1));
    if (status == HAL_OK) status = write_byte(reg);
    if (status == HAL_OK) status = start();
    if (status == HAL_OK) status = write_byte((uint8_t)((addr << 1) | 1U));
    for (uint16_t i = 0U; i < size && status == HAL_OK; ++i)
        status = read_byte(&data[i], (uint8_t)(i + 1U < size));
    return finish(status);
}
