#ifndef CAR_TASK4_H
#define CAR_TASK4_H
#include "car_task3.h"

typedef struct {
    uint8_t active, completed_laps; /* 运行标志、已在 A 点停稳完成的圈数。 */
    uint32_t start_ms, elapsed_ms; /* 四圈总起始时刻及累计用时，单位毫秒；圈间不清零。 */
    float ac_reference_deg; /* 首次启动保存的 AC 参考角，四圈始终共用。 */
} CarTask4;
/* 仅管理四圈流程，单圈控制继续复用 T20 任务三核心。 */
CarTask3Config CarTask4_Config(void);
int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input);
void CarTask4_Update(CarTask4 *task, CarTask3 *lap,
    const CarTask3Input *input, CarTask3Output *output);
void CarTask4_Cancel(CarTask4 *task, CarTask3 *lap);
#endif
