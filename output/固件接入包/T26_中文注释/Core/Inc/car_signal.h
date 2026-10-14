#ifndef CAR_SIGNAL_H
#define CAR_SIGNAL_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarSignal_Set(CarAppContext *app, int on);
void CarSignal_UpdateReady(CarAppContext *app, uint32_t now);
#endif
