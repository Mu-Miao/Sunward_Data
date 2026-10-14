#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"
#include <string.h>

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_Task3Config();
    /* The outer 160s watchdog owns the total; do not impose Task3's 40s
       contest limit independently on each lap of Task4. */
    config.deadline_ms = DPT_TASK4_LAP_WATCHDOG_MS;
    return config;
}

int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input)
{
    if (!task || !lap || !config || !input || task->active) return 0;
    if (!CarTask3_Start(lap, config, input)) return 0;
    memset(task, 0, sizeof *task);
    task->active = 1U;
    task->start_ms = input->now_ms;
    task->ac_reference_deg = lap->start_heading_deg;
    return 1;
}

void CarTask4_Cancel(CarTask4 *task, CarTask3 *lap)
{
    if (task) task->active = 0U;
    CarTask3_Cancel(lap);
}

void CarTask4_Update(CarTask4 *task, CarTask3 *lap,
    const CarTask3Input *input, CarTask3Output *output)
{
    if (!output) return;
    memset(output, 0, sizeof *output);
    if (!task || !lap || !input || !task->active) return;
    task->elapsed_ms = (uint32_t)(input->now_ms - task->start_ms);
    if (task->elapsed_ms >= DPT_TASK4_TIMEOUT_MS) {
        CarTask4_Cancel(task, lap);
        lap->fault = T3_TIMEOUT;
        output->reset_controllers = 1U;
        return;
    }
    CarTask3_Update(lap, input, output);
    if (lap->phase == T3_FAULT) { task->active = 0U; return; }
    if (lap->phase != T3_DONE) return;
    /* T20 reports DONE only at A after settled wheels; count exactly once. */
    ++task->completed_laps;
    if (task->completed_laps == DPT_TASK4_LAPS) { task->active = 0U; return; }
    CarTask3Config config = lap->config;
    config.start_from_ab = 1U; /* Reuse the existing turn+settle phases at A. */
    if (!CarTask3_Start(lap, &config, input)) {
        task->active = 0U;
        memset(output, 0, sizeof *output);
        output->reset_controllers = 1U;
        return;
    }
    /* Do not redefine AC from the latest endpoint yaw or emit A twice.
       Keep the just-completed lap's A pulse in output, with zero motion. */
    lap->start_heading_deg = task->ac_reference_deg;
    lap->pending_start_signal = 0U;
}
