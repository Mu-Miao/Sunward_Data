#ifndef CAR_LINE_H
#define CAR_LINE_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarLine_Read(CarAppContext *app);
void CarLine_Decode(CarAppContext *app);
void CarLine_Sample(CarAppContext *app, uint32_t now, int timing_ok);
void CarLine_SetTargets(CarAppContext *app, uint32_t now, float base);
#endif
