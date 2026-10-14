#ifndef CAR_UI_H
#define CAR_UI_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarUI_UpdateOLED(CarAppContext *app, uint32_t now, int enabled);
void CarUI_Log(CarAppContext *app, uint32_t now, int enabled);
#endif
