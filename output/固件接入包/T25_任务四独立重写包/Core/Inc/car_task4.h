#ifndef CAR_TASK4_H
#define CAR_TASK4_H
#include "car_task3.h"

typedef struct {
    uint8_t active, completed_laps;
    uint32_t start_ms, elapsed_ms;
    float ac_reference_deg;
} CarTask4;
/* Own only the four-lap lifecycle; the single-lap T20 core stays unchanged. */
CarTask3Config CarTask4_Config(void);
int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input);
void CarTask4_Update(CarTask4 *task, CarTask3 *lap,
    const CarTask3Input *input, CarTask3Output *output);
void CarTask4_Cancel(CarTask4 *task, CarTask3 *lap);
#endif
