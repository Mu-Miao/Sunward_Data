#ifndef CAR_WHEEL_H
#define CAR_WHEEL_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
void CarWheel_Clear(CarAppContext *app);
void CarWheel_Control(CarAppContext *app, unsigned i, float seconds);
int CarWheel_ConfigValid(void);
float CarWheel_Millimetres(unsigned wheel, int32_t counts);
void CarWheel_Measure(CarAppContext *app, const uint16_t count[2], float seconds);
#endif
