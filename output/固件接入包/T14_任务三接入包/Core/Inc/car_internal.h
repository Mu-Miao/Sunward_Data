#ifndef CAR_INTERNAL_H
#define CAR_INTERNAL_H
#include "car_app.h"
#include "car_config.h"
#include "car_math.h"
#include "car_state.h"
#include "car_wheel.h"
void CarApp_Stop(CarAppContext *app, TestState why, char faulty);
#include "car_button.h"
#include "car_route.h"
#include "car_task3_port.h"
#include "car_heading.h"
#include "car_line.h"
#include "car_command.h"
#include "car_signal.h"
#include "car_ui.h"
#include "car_power.h"

#ifdef CAR_APP_TEST
CarAppContext *CarApp_TestContext(void);
#endif
#endif
