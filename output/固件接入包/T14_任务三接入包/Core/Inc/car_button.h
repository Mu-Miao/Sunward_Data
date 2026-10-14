#ifndef CAR_BUTTON_H
#define CAR_BUTTON_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
KeyEvent CarButton_Poll(CarAppContext *app, uint32_t now, int enabled, int busy);
void CarButton_Handle(CarAppContext *app, uint32_t now, int enabled, uint8_t was_busy, KeyEvent key_pressed, Motion *requested, uint8_t *stop_requested);
#endif
