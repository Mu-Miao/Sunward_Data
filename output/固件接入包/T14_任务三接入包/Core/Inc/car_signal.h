#ifndef CAR_SIGNAL_H
#define CAR_SIGNAL_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarSignal_Set(CarAppContext *app, int on);
void CarSignal_UpdateReady(CarAppContext *app, uint32_t now);
#endif
