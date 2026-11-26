#include "buck_pwm.h"
#include "main.h"
#include "stm32g4xx_ll_tim.h"
#include <math.h>

#if BUCK_PWM_FREQUENCY_HZ == 0U || BUCK_PWM_DEADTIME_CODE == 0U || BUCK_PWM_DEADTIME_CODE > 255U
#error "PWM frequency must be positive and DeadTime must be a raw code from 1 to 255"
#endif

static uint32_t buck_pwm_initialized;
static uint32_t buck_pwm_period_ticks;
static uint32_t buck_pwm_dead_ticks;

static void BuckPWM_GPIO_HoldLow(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = BUCK_PWM_PA10_SPEED;
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Speed = BUCK_PWM_PB15_SPEED;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void BuckPWM_GPIO_ConnectTimer(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    /* Tune each pin's slew rate independently; timer settings stay fixed.
     * Recheck the measured edges/dead time after downloading this firmware.
     */
    gpio.Speed = BUCK_PWM_PA10_SPEED;
    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio);
#if !defined(BUCK_PWM_TEST_PA10_ONLY)
    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Speed = BUCK_PWM_PB15_SPEED;
    HAL_GPIO_Init(GPIOB, &gpio);
#else
    /* Diagnostic build: PB15 stays LOW. Retired PB13 is always a LOW GPIO. */
#endif
}

static uint32_t BuckPWM_DecodeDeadTime(uint32_t code)
{
    /* RM0440: BDTR.DTG encoding, CKD=DIV1 (one tick = 1 / TIM1CLK). */
    if (code < 0x80U)
    {
        return code;
    }
    if (code < 0xC0U)
    {
        return (64U + (code & 0x3FU)) * 2U;
    }
    if (code < 0xE0U)
    {
        return (32U + (code & 0x1FU)) * 8U;
    }
    return (32U + (code & 0x1FU)) * 16U;
}

void BuckPWM_Init(void)
{
    uint32_t timer_clock;
    uint32_t period_ticks;
    uint32_t dead_time_code;

    BuckPWM_Stop();
    buck_pwm_initialized = 0U;
    buck_pwm_period_ticks = 0U;
    BuckPWM_GPIO_HoldLow();

    timer_clock = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock *= 2U;
    }

    /* PSC=0 keeps PWM and dead-time clocks identical.
     * Require an exact integer period and an exact 50% compare value.
     */
    period_ticks = timer_clock / BUCK_PWM_FREQUENCY_HZ;
    if ((timer_clock % BUCK_PWM_FREQUENCY_HZ) != 0U ||
        period_ticks < 4U || period_ticks > 65536U ||
        (period_ticks & 1U) != 0U)
    {
        Error_Handler();
        return;
    }

    /* Match CubeMX's DeadTime field directly; validate decoded pulse width. */
    dead_time_code = BUCK_PWM_DEADTIME_CODE;
    if (BuckPWM_DecodeDeadTime(dead_time_code) >= period_ticks / 2U)
    {
        Error_Handler();
        return;
    }

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();

    LL_TIM_SetPrescaler(TIM1, 0U);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetRepetitionCounter(TIM1, 0U);
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_SetAutoReload(TIM1, period_ticks - 1U);

    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_SetCompareCH3(TIM1, period_ticks / 2U);

    /* Reset leaves all polarities active-high and all idle levels low.
     * CH3 and CH3N share OC3REF and the hardware dead-time generator.
     * The TIM1 reset above leaves CH1/CH1N disabled; no duplicated compare
     * channel is required. Only PA10/CH3 and PB15/CH3N are mapped to AF.
     */
    LL_TIM_SetOffStates(TIM1, LL_TIM_OSSI_ENABLE, LL_TIM_OSSR_ENABLE);
    LL_TIM_OC_SetDeadTime(TIM1, dead_time_code);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);

    /* Load CCR3 while the counter/MOE are off. */
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_ClearFlag_UPDATE(TIM1);
    BuckPWM_GPIO_ConnectTimer();
    buck_pwm_period_ticks = period_ticks;
    buck_pwm_dead_ticks = BuckPWM_DecodeDeadTime(dead_time_code);
    buck_pwm_initialized = 1U;
}

uint32_t BuckPWM_SetDuty(float duty)
{
    if (!buck_pwm_initialized || !isfinite(duty) || duty <= 0.0f || duty >= 1.0f)
    {
        BuckPWM_Stop();
        return 0U;
    }
    uint32_t compare = (uint32_t)(duty * (float)buck_pwm_period_ticks + 0.5f);
    if (compare <= buck_pwm_dead_ticks || compare >= buck_pwm_period_ticks - buck_pwm_dead_ticks)
    {
        BuckPWM_Stop();
        return 0U;
    }
    /* One preloaded CCR3 controls the complementary pair. A natural update
     * transfers it to both outputs; no UDIS manipulation or forced UG is
     * needed, so the running PWM phase/frequency stay unchanged. */
    LL_TIM_OC_SetCompareCH3(TIM1, compare);
    __DSB();
    return 1U;
}

uint32_t BuckPWM_IsRunning(void)
{
    return __HAL_RCC_TIM1_IS_CLK_ENABLED()
        && (READ_BIT(TIM1->CR1, TIM_CR1_CEN) != 0U)
        && (READ_BIT(TIM1->BDTR, TIM_BDTR_MOE) != 0U);
}

void BuckPWM_Start(void)
{
    uint32_t start_tick;

    if (buck_pwm_initialized == 0U)
    {
        Error_Handler();
        return;
    }

    BuckPWM_Stop();
    LL_TIM_SetCounter(TIM1, 0U);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_ClearFlag_UPDATE(TIM1);

    /* Start both outputs on the same hardware update, after one blank period.
     * AOE is used only for startup, then disabled to prevent auto-restart.
     */
    start_tick = HAL_GetTick();
    LL_TIM_EnableAutomaticOutput(TIM1);
    LL_TIM_EnableCounter(TIM1);
    while (LL_TIM_IsEnabledAllOutputs(TIM1) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= 10U)
        {
            BuckPWM_Stop();
            Error_Handler();
            return;
        }
    }
    LL_TIM_DisableAutomaticOutput(TIM1);
}

void BuckPWM_Stop(void)
{
    if (__HAL_RCC_TIM1_IS_CLK_ENABLED())
    {
        /* Clear AOE together with MOE so an update cannot re-enable outputs. */
        CLEAR_BIT(TIM1->BDTR, TIM_BDTR_AOE | TIM_BDTR_MOE);
        LL_TIM_DisableCounter(TIM1);
        __DSB();
    }
}
