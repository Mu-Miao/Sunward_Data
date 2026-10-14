#ifndef DUAL_PI_TEST_H
#define DUAL_PI_TEST_H
#include "car_config.h"
#include "car_app.h"
/* 兼容原工程的接口名称，新整车模块使用 car_app.h。 */
HAL_StatusTypeDef DualPITest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b));
HAL_StatusTypeDef DualPITest_EnableBluetooth(UART_HandleTypeDef *bluetooth);
void DualPITest_EnableHeading(int mpu_initialized);
void DualPITest_Poll(uint32_t tick10_count);
#endif
