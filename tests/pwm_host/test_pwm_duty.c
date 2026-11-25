/* Native register-model test. HAL/LL are simulated; no hardware is touched.
 * gcc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections
 * -Itests/pwm_host -ICore/Inc tests/pwm_host/test_pwm_duty.c
 * -Wl,--gc-sections -lm -o pwm_test */
#include "main.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../Core/Src/buck_pwm.c"

TestTimer test_timer;
TestRCC test_rcc;
uint32_t test_clock_enabled = 1U;
static uint32_t active1 = 4200U, active3 = 4200U, inject_stop;

static void Rollover(void)
{
    if ((test_timer.CR1 & TIM_CR1_UDIS) == 0U)
    {
        active1 = test_timer.CCR1;
        active3 = test_timer.CCR3;
    }
    assert(active1 == active3);
}
void LL_TIM_OC_SetCompareCH1(TestTimer *t, uint32_t value)
{
    t->CCR1 = value;
    Rollover(); /* Worst case: hardware rollover between both CCR writes. */
    if (inject_stop) BuckPWM_Stop();
}
void LL_TIM_OC_SetCompareCH3(TestTimer *t, uint32_t value) { t->CCR3 = value; Rollover(); }
void LL_TIM_DisableUpdateEvent(TestTimer *t) { t->CR1 |= TIM_CR1_UDIS; }
void LL_TIM_EnableUpdateEvent(TestTimer *t) { t->CR1 &= ~TIM_CR1_UDIS; }
void LL_TIM_DisableCounter(TestTimer *t) { t->CR1 &= ~TIM_CR1_CEN; }
void HAL_GPIO_WritePin(void *p, uint32_t pin, uint32_t value) { (void)p; (void)pin; (void)value; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 168000000U; }
uint32_t HAL_GetTick(void) { static uint32_t tick; return tick++; }
void Error_Handler(void) { assert(0); }
#define NOOP_ONE(name) void name(TestTimer *t) { (void)t; }
#define NOOP_TWO(name) void name(TestTimer *t, uint32_t v) { (void)t; (void)v; }
NOOP_TWO(LL_TIM_SetPrescaler)
NOOP_TWO(LL_TIM_SetCounterMode)
NOOP_TWO(LL_TIM_SetClockDivision)
NOOP_TWO(LL_TIM_SetRepetitionCounter)
NOOP_ONE(LL_TIM_EnableARRPreload)
NOOP_TWO(LL_TIM_SetAutoReload)
NOOP_TWO(LL_TIM_OC_EnablePreload)
NOOP_TWO(LL_TIM_OC_SetDeadTime)
NOOP_TWO(LL_TIM_CC_EnableChannel)
NOOP_ONE(LL_TIM_ClearFlag_UPDATE)
NOOP_TWO(LL_TIM_SetCounter)
void LL_TIM_OC_SetMode(TestTimer *t, uint32_t channel, uint32_t mode)
{ (void)t; (void)channel; (void)mode; }
void LL_TIM_SetOffStates(TestTimer *t, uint32_t idle, uint32_t run)
{ (void)t; (void)idle; (void)run; }
void LL_TIM_GenerateEvent_UPDATE(TestTimer *t) { (void)t; Rollover(); }
void LL_TIM_EnableAutomaticOutput(TestTimer *t) { t->BDTR |= TIM_BDTR_AOE; }
void LL_TIM_DisableAutomaticOutput(TestTimer *t) { t->BDTR &= ~TIM_BDTR_AOE; }
void LL_TIM_EnableCounter(TestTimer *t)
{
    t->CR1 |= TIM_CR1_CEN;
    if (t->BDTR & TIM_BDTR_AOE) t->BDTR |= TIM_BDTR_MOE;
}
uint32_t LL_TIM_IsEnabledAllOutputs(TestTimer *t) { return (t->BDTR & TIM_BDTR_MOE) != 0U; }

int main(void)
{
    assert(!BuckPWM_SetDuty(0.5f));
    buck_pwm_initialized = 1U;
    buck_pwm_period_ticks = 8400U;
    buck_pwm_dead_ticks = 320U;
    test_timer.CR1 = TIM_CR1_CEN;
    test_timer.BDTR = TIM_BDTR_MOE;
    test_timer.CCR1 = test_timer.CCR3 = 4200U;
    assert(BuckPWM_SetDuty(0.4f));
    assert(active1 == 4200U && active3 == 4200U);
    Rollover();
    assert(active1 == 3360U && active3 == 3360U && BuckPWM_IsRunning());
    test_timer.CR1 |= TIM_CR1_UDIS;
    assert(BuckPWM_SetDuty(0.65f));
    assert((test_timer.CR1 & TIM_CR1_UDIS) != 0U);
    assert(active1 == 3360U && active3 == 3360U);
    test_timer.CR1 &= ~TIM_CR1_UDIS;
    Rollover();
    assert(active1 == 5460U && active3 == 5460U);
    inject_stop = 1U;
    assert(BuckPWM_SetDuty(0.5f));
    assert(!BuckPWM_IsRunning()); /* A setter must never re-enable MOE/CEN. */
    inject_stop = 0U;
    assert(!BuckPWM_SetDuty(NAN));
    assert(!BuckPWM_SetDuty(0.0f));
    assert(!BuckPWM_SetDuty(1.0f));
    assert(!BuckPWM_SetDuty(0.01f));
    assert(!BuckPWM_SetDuty(0.99f));
    puts("PASS PWM: atomic CCR preload, preserved UDIS, no restart, invalid duty rejection");
    return 0;
}
