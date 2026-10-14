#ifndef LAB_SOFT_I2C_H
#define LAB_SOFT_I2C_H
#include "stm32f1xx_hal.h"

/* Foreground only. PB14=SCL, PB15=SDA; addresses are 7-bit. */
HAL_StatusTypeDef SoftI2C_Init(void);
HAL_StatusTypeDef SoftI2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef SoftI2C_ReadReg(uint8_t addr, uint8_t reg,
                              uint8_t *data, uint16_t size);
/* bit0=SCL, bit1=SDA; idle bus normally returns 3. */
uint8_t SoftI2C_Lines(void);
#endif
