#ifndef PWM_HOST_MAIN_H
#define PWM_HOST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t CR1, BDTR, CCR1, CCR3, PSC, ARR, CNT, CCER; } TestTimer;
typedef struct { uint32_t CFGR; } TestRCC;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
extern TestTimer test_timer;
extern TestRCC test_rcc;
extern uint32_t test_clock_enabled;
#define TIM1 (&test_timer)
#define RCC (&test_rcc)
#define GPIOA ((void *)1)
#define GPIOB ((void *)2)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_15 (1U << 15)
#define GPIO_PIN_RESET 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_AF_PP 2U
#define GPIO_PULLDOWN 2U
#define GPIO_SPEED_FREQ_MEDIUM 1U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_AF6_TIM1 6U
#define GPIO_AF4_TIM1 4U
#define RCC_CFGR_PPRE2 (7U << 11)
#define TIM_CR1_CEN 1U
#define TIM_CR1_UDIS 2U
#define TIM_BDTR_MOE (1U << 15)
#define TIM_BDTR_AOE (1U << 14)
#define READ_BIT(reg, bits) ((reg) & (bits))
#define CLEAR_BIT(reg, bits) ((reg) &= ~(bits))
#define __DSB() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() (test_clock_enabled = 1U)
#define __HAL_RCC_TIM1_IS_CLK_ENABLED() (test_clock_enabled)
#define __HAL_RCC_TIM1_FORCE_RESET() TestForceReset()
#define __HAL_RCC_TIM1_RELEASE_RESET() ((void)0)
void HAL_GPIO_WritePin(void *, uint32_t, uint32_t);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
uint32_t HAL_RCC_GetPCLK2Freq(void);
uint32_t HAL_GetTick(void);
void Error_Handler(void);
void TestForceReset(void);
#endif
