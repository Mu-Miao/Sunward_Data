#ifndef CAR_APP_H
#define CAR_APP_H
#include "stm32f1xx_hal.h"

/* 整车只有一个实例，仅在主循环中调用。
 * CubeMX 外设、PWM 和编码器初始化后调用 Begin，初始化期间电机保持停止。
 * 主循环持续调用 Poll，传入 TIM2 每 10 毫秒累加的计数。
 * 中断只负责串口数据入队和定时计数，不在中断内执行整车控制。 */
HAL_StatusTypeDef CarApp_Begin(UART_HandleTypeDef *uart,
    TIM_HandleTypeDef *left_encoder, TIM_HandleTypeDef *right_encoder,
    ADC_HandleTypeDef *battery_adc, void (*motor_ab)(int a, int b));
HAL_StatusTypeDef CarApp_EnableBluetooth(UART_HandleTypeDef *bluetooth);
void CarApp_EnableHeading(int mpu_initialized);
void CarApp_Poll(uint32_t tick10_count);
#endif
