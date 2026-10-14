#ifndef CAR_UI_H
#define CAR_UI_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarUI_UpdateOLED(CarAppContext *app, uint32_t now, int enabled);
void CarUI_Log(CarAppContext *app, uint32_t now, int enabled);
#endif
