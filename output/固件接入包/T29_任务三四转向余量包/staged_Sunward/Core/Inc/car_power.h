#ifndef CAR_POWER_H
#define CAR_POWER_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarPower_Poll(CarAppContext *app, uint32_t now);
#endif
