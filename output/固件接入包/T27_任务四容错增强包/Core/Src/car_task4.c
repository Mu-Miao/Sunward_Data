/* 任务四独立管理四圈计数、总超时和 A 点接续；单圈运动复用任务三核心。 */
#include "car_task4.h"
#include "car_route.h"
#include "car_config.h"
#include <string.h>

CarTask3Config CarTask4_Config(void)
{
    CarTask3Config config = CarRoute_Task3Config();
    /* 外层统一执行四圈总计 160 秒的超时保护。
       不将任务三的 40 秒比赛时限单独施加到任务四每一圈。 */
    config.deadline_ms = DPT_TASK4_LAP_WATCHDOG_MS;
    config.lost_ms = DPT_TASK4_LOST_MS;
    config.recovery_ms = DPT_TASK4_RECOVERY_MS;
    return config;
}

int CarTask4_Start(CarTask4 *task, CarTask3 *lap,
    const CarTask3Config *config, const CarTask3Input *input)
{
    if (!task || !lap || !config || !input || task->active) return 0;
    if (!CarTask3_Start(lap, config, input)) return 0;
    /* 新任务启动时清圈数；只有成功启动单圈核心后才进入运行状态。 */
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
    /* 每次先清空输出，缺少输入或任务已停止时不会保留旧电机目标。 */
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
    if (lap->phase == T3_ARC) {
        if (output->control == T3_HALT) {
            task->resume_pending = 1U;
            task->resume_active = 0U;
        } else if (output->control == T3_FOLLOW_LINE && task->resume_pending) {
            if (!task->resume_active) { task->resume_active = 1U; task->resume_ms = input->now_ms; }
            float ramp = DPT_TASK4_RESUME_MMPS + DPT_TASK4_RESUME_ACCEL_MMPS2 *
                (float)(uint32_t)(input->now_ms - task->resume_ms) / 1000.0f;
            if (output->base_mmps > ramp) output->base_mmps = ramp;
            if (ramp >= lap->config.arc_mmps) task->resume_pending = task->resume_active = 0U;
        }
    } else task->resume_pending = task->resume_active = 0U;
    if (lap->phase != T3_DONE) return;
    /* T20 单圈核心仅在 A 点两轮停稳后报告完成，此时圈数只增加一次。 */
    ++task->completed_laps;
    if (task->completed_laps == DPT_TASK4_LAPS) { task->active = 0U; return; }
    CarTask3Config config = lap->config;
    config.start_from_ab = 1U; /* 在 A 点复用已有的陀螺仪转向及停稳阶段。 */
    if (!CarTask3_Start(lap, &config, input)) {
        task->active = 0U;
        memset(output, 0, sizeof *output);
        output->reset_controllers = 1U;
        return;
    }
    /* 继续使用首次启动时保存的 AC 参考，不依据本圈终点角度重新定义方向。
       保留本圈结束的 A 点提示及零运动输出，禁止下一圈重复发出启动 A 提示。 */
    lap->start_heading_deg = task->ac_reference_deg;
    lap->pending_start_signal = 0U;
}
