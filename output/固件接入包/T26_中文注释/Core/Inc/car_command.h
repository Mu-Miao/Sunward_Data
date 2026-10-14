#ifndef CAR_COMMAND_H
#define CAR_COMMAND_H
#include "car_state.h"

/* 主循环内部模块接口；状态的初始化与生命周期由 CarApp 统一管理。 */
extern const char CarCommand_Banner[159];
void CarCommand_Poll(CarAppContext *app, uint32_t now, Motion *requested, uint8_t *stop_requested);
#endif
