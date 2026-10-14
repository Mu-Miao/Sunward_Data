#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_Task3Config();
    config.laps = DPT_TASK4_LAPS;
    config.deadline_ms = DPT_TASK4_TIMEOUT_MS;
    return config;
}
