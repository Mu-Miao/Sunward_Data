#ifndef CAR_TASK3_PORT_H
#define CAR_TASK3_PORT_H
#include "car_task3.h"
#include "car_state.h"

/* T13 adapter. The scheduler still owns start, stop, sampling and PWM commit. */
CarTask3Input CarTask3Port_Input(const CarAppContext *app, uint32_t now,
    int enabled, int stop_requested, int protection_fault);
/* Call only after checking route.phase, all protection gates and enable.
 * HALT clears both PI controllers. No function commits a positive PWM. */
void CarTask3Port_Apply(CarAppContext *app, const CarTask3Output *output,
    uint32_t now);
TestState CarTask3Port_StopState(CarTask3Fault fault);
#endif
