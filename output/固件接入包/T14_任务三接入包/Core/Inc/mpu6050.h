#ifndef LAB_MPU6050_H
#define LAB_MPU6050_H
#include "stm32f1xx_hal.h"

typedef struct {
    uint8_t address;   /* 7-bit bus address: 0x68 or 0x69 */
    uint8_t who;
    uint8_t power;
    uint8_t divider;
    uint8_t filter;
    uint8_t gyro_config;
    uint8_t accel_config;
} MPU6050_Info;
typedef struct {
    int16_t ax, ay, az;
    int16_t temperature;
    int16_t gx, gy, gz;
} MPU6050_Raw;

/* Foreground only; Init waits for reset and sensor settling. */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Info *info);
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw *raw);
#endif
