#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_Task3Config();
    config.arc_mmps = DPT_TASK4_ARC_MMPS;
    config.arc_acceleration_mmps2 = DPT_TASK4_ARC_ACCEL_MMPS2;
    config.laps = DPT_TASK4_LAPS;
    config.deadline_ms = DPT_TASK4_TIMEOUT_MS;
    return config;
}
