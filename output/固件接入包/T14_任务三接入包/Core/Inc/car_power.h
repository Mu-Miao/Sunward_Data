#ifndef CAR_POWER_H
#define CAR_POWER_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarPower_Poll(CarAppContext *app, uint32_t now);
#endif
