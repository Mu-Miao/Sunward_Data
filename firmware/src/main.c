#include "stm32f1xx.h"

/*
 * C10B minimum motor test
 *
 * Motor A: TIM3_CH1 (PA6), TIM3_CH2 (PA7)
 * Motor B: TIM3_CH3 (PB0), TIM3_CH4 (PB1)
 *
 * Safety sequence after every reset:
 *   1. Brake for 2 seconds.
 *   2. Run both motors slowly for 2 seconds.
 *   3. Brake forever.
 */

enum {
    PWM_TOP = 7200,
    TEST_PWM = 1400,
};

void SystemInit(void)
{

}

/* This bare-metal test has no C++ constructors, so no C runtime is needed. */
void __libc_init_array(void)
{
}

static void delay_init(void)
{
    /* 8 MHz / 8000 = one SysTick event per millisecond. */
    SysTick->LOAD = 8000U - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U) {
        }
    }
}

static void motor_set(int motor_a, int motor_b)
{
    if (motor_a > PWM_TOP) motor_a = PWM_TOP;
    if (motor_a < -PWM_TOP) motor_a = -PWM_TOP;
    if (motor_b > PWM_TOP) motor_b = PWM_TOP;
    if (motor_b < -PWM_TOP) motor_b = -PWM_TOP;

    if (motor_a < 0) {
        TIM3->CCR1 = PWM_TOP;
        TIM3->CCR2 = (uint32_t)(PWM_TOP + motor_a);
    } else {
        TIM3->CCR1 = (uint32_t)(PWM_TOP - motor_a);
        TIM3->CCR2 = PWM_TOP;
    }

    if (motor_b < 0) {
        TIM3->CCR3 = PWM_TOP;
        TIM3->CCR4 = (uint32_t)(PWM_TOP + motor_b);
    } else {
        TIM3->CCR3 = (uint32_t)(PWM_TOP - motor_b);
        TIM3->CCR4 = PWM_TOP;
    }
}

static void motor_stop(void)
{
    motor_set(0, 0);
}

static void motor_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* PA6 and PA7: alternate-function push-pull, 50 MHz. */
    GPIOA->CRL &= ~((0xFU << 24U) | (0xFU << 28U));
    GPIOA->CRL |=  ((0xBU << 24U) | (0xBU << 28U));

    /* PB0 and PB1: alternate-function push-pull, 50 MHz. */
    GPIOB->CRL &= ~((0xFU << 0U) | (0xFU << 4U));
    GPIOB->CRL |=  ((0xBU << 0U) | (0xBU << 4U));

    TIM3->PSC = 0U;
    TIM3->ARR = PWM_TOP - 1U;

    TIM3->CCMR1 = TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCMR2 = TIM_CCMR2_OC3PE |
                  TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4PE |
                  TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    motor_stop();

    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E |
                 TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

int main(void)
{
    delay_init();
    motor_init();

    delay_ms(2000U);
    motor_set(TEST_PWM, TEST_PWM);
    delay_ms(2000U);
    motor_stop();

    for (;;) {
        __WFI();
    }
}
