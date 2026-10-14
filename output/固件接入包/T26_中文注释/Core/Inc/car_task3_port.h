#ifndef CAR_TASK3_PORT_H
#define CAR_TASK3_PORT_H
#include "car_task3.h"
#include "car_state.h"

/* 单圈路线与整车模块的适配接口；启动、停车、采样和 PWM 提交由调度器负责。 */
CarTask3Input CarTask3Port_Input(const CarAppContext *app, uint32_t now,
    int enabled, int stop_requested, int protection_fault);
/* 确认路线阶段、各项保护及使能状态后才调用。
 * HALT 清除两轮 PI 状态；本接口不直接提交正向 PWM。 */
void CarTask3Port_Apply(CarAppContext *app, const CarTask3Output *output,
    uint32_t now);
TestState CarTask3Port_StopState(CarTask3Fault fault);
#endif
