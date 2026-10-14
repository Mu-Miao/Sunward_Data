#include "mpu6050.h"
#include "soft_i2c.h"

static uint8_t address;
static uint8_t ready;

static int16_t signed16(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 8) | data[1];
    int32_t result = value < 32768U ? (int32_t)value : (int32_t)value - 65536;
    return (int16_t)result;
}
static HAL_StatusTypeDef read_reg(uint8_t reg, uint8_t *value)
{
    return SoftI2C_ReadReg(address, reg, value, 1U);
}
HAL_StatusTypeDef MPU6050_Init(MPU6050_Info *info)
{
    if (info == NULL) return HAL_ERROR;
    ready = 0U;
    *info = (MPU6050_Info){0};
    HAL_StatusTypeDef status = SoftI2C_Init();
    if (status != HAL_OK) return status;
    HAL_Delay(100U);
    uint8_t found = 0U;
    for (uint8_t candidate = 0x68U; candidate <= 0x69U; ++candidate) {
        uint8_t who = 0U;
        status = SoftI2C_ReadReg(candidate, 0x75U, &who, 1U);
        info->address = candidate;
        info->who = who;
        if (status == HAL_OK && who == 0x68U) {
            address = candidate;
            found = 1U;
            break;
        }
    }
    if (!found) return status == HAL_OK ? HAL_ERROR : status;
    status = SoftI2C_WriteReg(address, 0x6BU, 0x80U); /* 复位 MPU6050 设备。 */
    if (status != HAL_OK) return status;
    HAL_Delay(100U);
    /* 唤醒设备并选 X 轴陀螺仪 PLL 为时钟；启用所有轴。
       采样率 100 Hz，低通配置 DLPF=3，陀螺仪量程正负 250 度/秒，加速度量程正负 2g。 */
    static const uint8_t registers[][2] = {
        {0x6BU, 0x01U}, {0x6CU, 0x00U}, {0x1AU, 0x03U},
        {0x19U, 0x09U}, {0x1BU, 0x00U}, {0x1CU, 0x00U},
        {0x38U, 0x00U}, {0x23U, 0x00U}, {0x6AU, 0x00U}
    };
    for (unsigned i = 0U; i < sizeof registers / sizeof registers[0]; ++i) {
        status = SoftI2C_WriteReg(address, registers[i][0], registers[i][1]);
        if (status != HAL_OK) return status;
    }
    HAL_Delay(100U);
    if ((status = read_reg(0x75U, &info->who)) != HAL_OK) return status;
    if ((status = read_reg(0x6BU, &info->power)) != HAL_OK) return status;
    if ((status = read_reg(0x19U, &info->divider)) != HAL_OK) return status;
    if ((status = read_reg(0x1AU, &info->filter)) != HAL_OK) return status;
    if ((status = read_reg(0x1BU, &info->gyro_config)) != HAL_OK) return status;
    if ((status = read_reg(0x1CU, &info->accel_config)) != HAL_OK) return status;
    if (info->who != 0x68U || info->power != 0x01U || info->divider != 9U ||
        info->filter != 3U || info->gyro_config != 0U || info->accel_config != 0U)
        return HAL_ERROR;
    ready = 1U;
    return HAL_OK;
}
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw *raw)
{
    if (!ready || raw == NULL) return HAL_ERROR;
    uint8_t data[14];
    HAL_StatusTypeDef status = SoftI2C_ReadReg(address, 0x3BU, data, sizeof data);
    if (status != HAL_OK) return status;
    MPU6050_Raw value = {
        signed16(&data[0]), signed16(&data[2]), signed16(&data[4]),
        signed16(&data[6]), signed16(&data[8]), signed16(&data[10]),
        signed16(&data[12])
    };
    *raw = value;
    return HAL_OK;
}
