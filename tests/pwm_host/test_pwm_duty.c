/* Native register-model test of the REAL PWM module. This verifies GPIO/AF
 * selection, channel ownership, preload/start/stop behavior and parameters;
 * it cannot validate physical MOSFET dead time or analog reference noise.
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/pwm_host -ICore/Inc
 * tests/pwm_host/test_pwm_duty.c -lm -o pwm_test */
#include "main.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../Core/Src/buck_pwm.c"

TestTimer test_timer;
TestRCC test_rcc;
uint32_t test_clock_enabled;
static uint32_t active3, inject_stop, inject_rollover, generated_updates;
static uint32_t preload_channels, mode_channels, dead_time_code, pb13_af_calls;
static uint32_t latch_a = 0xffffU, latch_b = 0xffffU;
static GPIO_InitTypeDef config_a[16], config_b[16];

static void Rollover(void)
{
    if ((test_timer.CR1 & TIM_CR1_UDIS) == 0U)
    {
        active3 = test_timer.CCR3;
    }
}
void LL_TIM_OC_SetCompareCH1(TestTimer *t, uint32_t value)
{
    (void)t; (void)value;
    assert(0); /* Retired cross-channel implementation must never be used. */
}
void LL_TIM_OC_SetCompareCH3(TestTimer *t, uint32_t value)
{
    if (inject_rollover) Rollover(); /* Boundary just before the atomic write. */
    t->CCR3 = value;
    if (inject_stop) BuckPWM_Stop();
}
void LL_TIM_DisableUpdateEvent(TestTimer *t) { t->CR1 |= TIM_CR1_UDIS; }
void LL_TIM_EnableUpdateEvent(TestTimer *t) { t->CR1 &= ~TIM_CR1_UDIS; }
void LL_TIM_DisableCounter(TestTimer *t) { t->CR1 &= ~TIM_CR1_CEN; }
void HAL_GPIO_WritePin(void *p, uint32_t pin, uint32_t value)
{
    assert(value == GPIO_PIN_RESET);
    assert(p == GPIOA || p == GPIOB);
    if (p == GPIOA) latch_a &= ~pin;
    else latch_b &= ~pin;
}
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g)
{
    assert(p == GPIOA || p == GPIOB);
    GPIO_InitTypeDef *config = p == GPIOA ? config_a : config_b;
    for (uint32_t bit = 0U; bit < 16U; ++bit)
        if (g->Pin & (1U << bit)) config[bit] = *g;
    if (p == GPIOB && (g->Pin & GPIO_PIN_13) && g->Mode == GPIO_MODE_AF_PP)
        ++pb13_af_calls;
}
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 168000000U; }
uint32_t HAL_GetTick(void) { static uint32_t tick; return tick++; }
void Error_Handler(void) { assert(0); }
#define NOOP_ONE(name) void name(TestTimer *t) { (void)t; }
#define NOOP_TWO(name) void name(TestTimer *t, uint32_t v) { (void)t; (void)v; }
void TestForceReset(void)
{
    memset(&test_timer, 0, sizeof(test_timer));
    preload_channels = mode_channels = dead_time_code = active3 = 0U;
}
void LL_TIM_SetPrescaler(TestTimer *t, uint32_t v) { t->PSC = v; }
NOOP_TWO(LL_TIM_SetCounterMode)
NOOP_TWO(LL_TIM_SetClockDivision)
NOOP_TWO(LL_TIM_SetRepetitionCounter)
NOOP_ONE(LL_TIM_EnableARRPreload)
void LL_TIM_SetAutoReload(TestTimer *t, uint32_t v) { t->ARR = v; }
void LL_TIM_OC_EnablePreload(TestTimer *t, uint32_t v) { (void)t; preload_channels |= v; }
void LL_TIM_OC_SetDeadTime(TestTimer *t, uint32_t v) { (void)t; dead_time_code = v; }
void LL_TIM_CC_EnableChannel(TestTimer *t, uint32_t v) { t->CCER |= v; }
NOOP_ONE(LL_TIM_ClearFlag_UPDATE)
void LL_TIM_SetCounter(TestTimer *t, uint32_t v) { t->CNT = v; }
void LL_TIM_OC_SetMode(TestTimer *t, uint32_t channel, uint32_t mode)
{ (void)t; assert(mode == LL_TIM_OCMODE_PWM1); mode_channels |= channel; }
void LL_TIM_SetOffStates(TestTimer *t, uint32_t idle, uint32_t run)
{ (void)t; assert(idle == LL_TIM_OSSI_ENABLE && run == LL_TIM_OSSR_ENABLE); }
void LL_TIM_GenerateEvent_UPDATE(TestTimer *t) { (void)t; ++generated_updates; Rollover(); }
void LL_TIM_EnableAutomaticOutput(TestTimer *t) { t->BDTR |= TIM_BDTR_AOE; }
void LL_TIM_DisableAutomaticOutput(TestTimer *t) { t->BDTR &= ~TIM_BDTR_AOE; }
void LL_TIM_EnableCounter(TestTimer *t)
{
    t->CR1 |= TIM_CR1_CEN;
    if (t->BDTR & TIM_BDTR_AOE) { Rollover(); t->BDTR |= TIM_BDTR_MOE; }
}
uint32_t LL_TIM_IsEnabledAllOutputs(TestTimer *t) { return (t->BDTR & TIM_BDTR_MOE) != 0U; }

int main(void)
{
    assert(!BuckPWM_SetDuty(0.5f));
    BuckPWM_Init();
    assert(buck_pwm_initialized && buck_pwm_period_ticks == 8400U && buck_pwm_dead_ticks == 320U);
    assert(test_timer.PSC == 0U && test_timer.ARR == 8399U && test_timer.CCR3 == 4200U);
    assert(168000000U / (test_timer.PSC + 1U) / (test_timer.ARR + 1U) == 20000U);
    assert(dead_time_code == 200U);
    assert(mode_channels == LL_TIM_CHANNEL_CH3 && preload_channels == LL_TIM_CHANNEL_CH3);
    assert(test_timer.CCER == (LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N));
    assert(test_timer.CCR1 == 0U && !BuckPWM_IsRunning());
    assert(config_a[10].Mode == GPIO_MODE_AF_PP && config_a[10].Alternate == GPIO_AF6_TIM1);
#if defined(BUCK_PWM_TEST_PA10_ONLY)
    assert(config_b[15].Mode == GPIO_MODE_OUTPUT_PP);
#else
    assert(config_b[15].Mode == GPIO_MODE_AF_PP && config_b[15].Alternate == GPIO_AF4_TIM1);
#endif
    assert(config_b[13].Mode == GPIO_MODE_OUTPUT_PP && pb13_af_calls == 0U);
    assert((latch_a & GPIO_PIN_10) == 0U && (latch_b & (GPIO_PIN_13 | GPIO_PIN_15)) == 0U);
    BuckPWM_Start();
    assert(BuckPWM_IsRunning() && (test_timer.BDTR & TIM_BDTR_AOE) == 0U);
    test_timer.CCR1 = 1234U; /* Prove runtime duty changes cannot touch CH1. */
    uint32_t updates = generated_updates;
    inject_rollover = 1U;
    assert(BuckPWM_SetDuty(0.4f));
    assert(active3 == 4200U && test_timer.CCR3 == 3360U && test_timer.CCR1 == 1234U);
    assert(generated_updates == updates && test_timer.ARR == 8399U);
    Rollover();
    assert(active3 == 3360U && BuckPWM_IsRunning());
    test_timer.CR1 |= TIM_CR1_UDIS;
    assert(BuckPWM_SetDuty(0.65f));
    assert((test_timer.CR1 & TIM_CR1_UDIS) != 0U);
    assert(active3 == 3360U);
    test_timer.CR1 &= ~TIM_CR1_UDIS;
    Rollover();
    assert(active3 == 5460U);
    inject_stop = 1U;
    assert(BuckPWM_SetDuty(0.5f));
    assert(!BuckPWM_IsRunning()); /* A setter must never re-enable MOE/CEN. */
    inject_stop = 0U;
    assert(!BuckPWM_SetDuty(NAN));
    assert(!BuckPWM_SetDuty(0.0f));
    assert(!BuckPWM_SetDuty(1.0f));
    assert(!BuckPWM_SetDuty(0.01f));
    assert(!BuckPWM_SetDuty(0.99f));
    assert(!BuckPWM_SetDuty(320.0f / 8400.0f));
    assert(!BuckPWM_SetDuty(8080.0f / 8400.0f));
    assert(BuckPWM_SetDuty(0.08f) && !BuckPWM_IsRunning());
    assert(BuckPWM_SetDuty(0.80f) && !BuckPWM_IsRunning());
    assert(test_timer.CCR1 == 1234U && pb13_af_calls == 0U);
    puts("PASS PWM: PA10 AF6/PB15 AF4, retired PB13 LOW, CH3/CH3N only, 20 kHz, DTG200");
    puts("PASS PWM: single CCR3 preload, no forced update/restart, preserved UDIS, duty limits");
    return 0;
}
