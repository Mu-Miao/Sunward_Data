#ifndef CAR_LINE_H
#define CAR_LINE_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarLine_Read(CarAppContext *app);
void CarLine_Decode(CarAppContext *app);
void CarLine_Sample(CarAppContext *app, uint32_t now, int timing_ok);
void CarLine_SetTargets(CarAppContext *app, uint32_t now, float base);
#endif
