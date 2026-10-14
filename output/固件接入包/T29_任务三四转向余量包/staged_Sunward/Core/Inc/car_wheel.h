#ifndef CAR_WHEEL_H
#define CAR_WHEEL_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
void CarWheel_Clear(CarAppContext *app);
void CarWheel_Control(CarAppContext *app, unsigned i, float seconds);
int CarWheel_ConfigValid(void);
float CarWheel_Millimetres(unsigned wheel, int32_t counts);
void CarWheel_Measure(CarAppContext *app, const uint16_t count[2], float seconds);
#endif
