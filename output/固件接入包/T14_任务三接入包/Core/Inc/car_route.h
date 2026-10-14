#ifndef CAR_ROUTE_H
#define CAR_ROUTE_H
#include "car_state.h"

/* Internal foreground module API; state lifetime is owned by CarApp. */
CarTask3Config CarRoute_Task3Config(void);
int CarRoute_IsDistance(Motion motion);
void CarRoute_Select(CarAppContext *app, Motion selected, uint32_t now);
void CarRoute_Request(CarAppContext *app, Motion requested, uint32_t now);
void CarRoute_UpdateTask2(CarAppContext *app, uint32_t now);
int CarRoute_IsArc(CarAppContext *app);
int CarRoute_UsesLine(CarAppContext *app);
int CarRoute_UsesHeading(CarAppContext *app);
void CarRoute_Update(CarAppContext *app, uint32_t now, const uint16_t count[2]);
#endif
