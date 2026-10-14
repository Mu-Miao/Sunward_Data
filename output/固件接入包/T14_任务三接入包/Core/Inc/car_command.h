#ifndef CAR_COMMAND_H
#define CAR_COMMAND_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
extern const char CarCommand_Banner[152];
void CarCommand_Poll(CarAppContext *app, uint32_t now, Motion *requested, uint8_t *stop_requested);
#endif
