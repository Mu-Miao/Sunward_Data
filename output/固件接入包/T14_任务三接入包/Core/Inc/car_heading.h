#ifndef CAR_HEADING_H
#define CAR_HEADING_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarHeading_Enable(CarAppContext *app, int mpu_initialized);
void CarHeading_BeginCalibration(CarAppContext *app, uint32_t now, int enabled);
void CarHeading_Sample(CarAppContext *app, uint32_t now, int enabled, int wheels_moved);
#endif
