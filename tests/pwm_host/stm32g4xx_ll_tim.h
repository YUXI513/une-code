#ifndef PWM_HOST_LL_TIM_H
#define PWM_HOST_LL_TIM_H
#include "main.h"
#define LL_TIM_COUNTERMODE_UP 0U
#define LL_TIM_CLOCKDIVISION_DIV1 0U
#define LL_TIM_CHANNEL_CH1 1U
#define LL_TIM_CHANNEL_CH1N 2U
#define LL_TIM_CHANNEL_CH3 4U
#define LL_TIM_CHANNEL_CH3N 8U
#define LL_TIM_OCMODE_PWM1 1U
#define LL_TIM_OSSI_ENABLE 1U
#define LL_TIM_OSSR_ENABLE 1U
void LL_TIM_SetPrescaler(TestTimer *, uint32_t);
void LL_TIM_SetCounterMode(TestTimer *, uint32_t);
void LL_TIM_SetClockDivision(TestTimer *, uint32_t);
void LL_TIM_SetRepetitionCounter(TestTimer *, uint32_t);
void LL_TIM_EnableARRPreload(TestTimer *);
void LL_TIM_SetAutoReload(TestTimer *, uint32_t);
void LL_TIM_OC_SetMode(TestTimer *, uint32_t, uint32_t);
void LL_TIM_OC_EnablePreload(TestTimer *, uint32_t);
void LL_TIM_OC_SetCompareCH1(TestTimer *, uint32_t);
void LL_TIM_OC_SetCompareCH3(TestTimer *, uint32_t);
void LL_TIM_SetOffStates(TestTimer *, uint32_t, uint32_t);
void LL_TIM_OC_SetDeadTime(TestTimer *, uint32_t);
void LL_TIM_CC_EnableChannel(TestTimer *, uint32_t);
void LL_TIM_GenerateEvent_UPDATE(TestTimer *);
void LL_TIM_ClearFlag_UPDATE(TestTimer *);
void LL_TIM_SetCounter(TestTimer *, uint32_t);
void LL_TIM_EnableAutomaticOutput(TestTimer *);
void LL_TIM_EnableCounter(TestTimer *);
uint32_t LL_TIM_IsEnabledAllOutputs(TestTimer *);
void LL_TIM_DisableAutomaticOutput(TestTimer *);
void LL_TIM_DisableCounter(TestTimer *);
void LL_TIM_DisableUpdateEvent(TestTimer *);
void LL_TIM_EnableUpdateEvent(TestTimer *);
#endif
