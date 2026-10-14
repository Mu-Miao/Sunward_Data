#include "timer.h"


/**
 * @brief  初始化TIM7定时器，配置为2ms定时中断
 * @param  无
 * @retval 无
 */
void TIM7_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* 使能TIM7时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    
    /* 
     * 定时器配置
     * STM32F407 APB1定时器时钟为84MHz
     * 定时器频率 = TIM_CLK / (Prescaler + 1) / (Period + 1)
     * 2ms = 0.002s，频率 = 500Hz
     * 设置: Prescaler = 8399, Period = 19
     * 实际频率 = 84000000 / 8400 / 20 = 500Hz = 2ms
     */
    TIM_TimeBaseStructure.TIM_Period = 20 - 1;              // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 8400 - 1;         // 预分频系数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    /* 使能TIM7更新中断 */
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    /* 配置TIM7中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 使能TIM7 */
    TIM_Cmd(TIM7, ENABLE);
}


