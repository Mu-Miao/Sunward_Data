#ifndef CAR_HEADING_H
#define CAR_HEADING_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarHeading_Enable(CarAppContext *app, int mpu_initialized);
void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled);
void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved);
#endif
