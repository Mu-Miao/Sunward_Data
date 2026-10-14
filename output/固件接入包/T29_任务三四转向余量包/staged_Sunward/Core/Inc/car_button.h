#ifndef CAR_BUTTON_H
#define CAR_BUTTON_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
KeyEvent CarButton_Poll(CarAppContext *app, uint32_t now, int enabled, int busy);
void CarButton_Handle(CarAppContext *app, uint32_t now, int enabled, uint8_t was_busy, KeyEvent key_pressed, Motion *requested, uint8_t *stop_requested);
#endif
