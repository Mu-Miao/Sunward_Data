#ifndef LAB_SOFT_I2C_H
#define LAB_SOFT_I2C_H
#include "stm32f1xx_hal.h"

/* 软件 I2C 仅在主循环使用：PB14 为 SCL，PB15 为 SDA；地址采用七位格式。 */
HAL_StatusTypeDef SoftI2C_Init(void);
HAL_StatusTypeDef SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef SoftI2C_ReadReg(uint8_t addr, uint8_t reg,
                              uint8_t *data, uint16_t size);
/* 位 0 对应 SCL，位 1 对应 SDA；总线空闲且两线都为高电平时返回 3。 */
uint8_t SoftI2C_Lines(void);
#endif
