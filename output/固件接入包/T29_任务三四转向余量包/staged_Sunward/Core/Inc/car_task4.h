#ifndef CAR_TASK4_H
#define CAR_TASK4_H
#include "car_task3.h"

typedef struct {
    uint8_t active, completed_laps; /* 运行标志、已在 A 点停稳完成的圈数。 */
    uint32_t start_ms, elapsed_ms; /* 四圈总起始时刻及累计用时，单位毫秒；圈间不清零。 */
    uint8_t resume_pending, resume_active;
    uint32_t resume_ms; /* 失线恢复加速的独立起点，不重置阶段、里程及总超时。 */
    float ac_reference_deg; /* 首次启动保存的 AC 参考角，四圈始终共用。 */
} CarTask4;
/* 管理四圈流程及恢复加速；复用任务三、四共用的单圈与受限寻线核心。 */
CarTask3Config CarTask4_Config(void);
int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input);
void CarTask4_Update(CarTask4 *task, CarTask3 *lap,
    const CarTask3Input *input, CarTask3Output *output);
void CarTask4_Cancel(CarTask4 *task, CarTask3 *lap);
#endif
