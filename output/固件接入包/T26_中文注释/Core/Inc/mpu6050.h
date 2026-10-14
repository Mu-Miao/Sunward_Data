#ifndef LAB_MPU6050_H
#define LAB_MPU6050_H
#include "stm32f1xx_hal.h"

typedef struct {
    uint8_t address;   /* I2C 七位设备地址，取 0x68 或 0x69。 */
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

/* 仅在主循环中调用；初始化会等待设备复位及传感器稳定。 */
HAL_StatusTypeDef MPU6050_Init(MPU6050_Info *info);
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Raw *raw);
#endif
