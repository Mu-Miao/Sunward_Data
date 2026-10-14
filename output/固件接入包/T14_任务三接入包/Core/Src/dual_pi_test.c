#include "dual_pi_test.h"

/* Compatibility only. All control logic lives in separately compiled modules. */
HAL_StatusTypeDef DualPITest_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b))
{
    return CarApp_Begin(uart, left_encoder, right_encoder, battery_adc, motor_ab);
}
HAL_StatusTypeDef DualPITest_EnableBluetooth(UART_HandleTypeDef *bluetooth)
{
    return CarApp_EnableBluetooth(bluetooth);
}
void DualPITest_EnableHeading(int mpu_initialized)
{
    CarApp_EnableHeading(mpu_initialized);
}
void DualPITest_Poll(uint32_t tick10_count)
{
    CarApp_Poll(tick10_count);
}
