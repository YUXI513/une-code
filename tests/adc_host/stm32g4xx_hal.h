#ifndef TEST_HAL_H
#define TEST_HAL_H
/* Host-only HAL double. Never put this directory on the firmware include path. */
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t ISR; } ADC_TypeDef;
typedef struct { uint32_t CSR; } VREFBUF_TypeDef;
extern ADC_TypeDef test_adc;
extern VREFBUF_TypeDef test_vrefbuf;
#define ADC1 (&test_adc)
#define VREFBUF (&test_vrefbuf)
#define VREFBUF_CSR_ENVR 1U
#define VREFBUF_CSR_HIZ 2U
#define VREFBUF_CSR_VRR 8U
#define VREFBUF_CSR_VRS 0x30U
#define SYSCFG_VREFBUF_VOLTAGE_SCALE1 0x10U
#define SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE VREFBUF_CSR_HIZ
#define SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE 0U
#define GPIOA ((void *)1)
typedef struct { uint32_t PeriphClockSelection, Adc12ClockSelection; } RCC_PeriphCLKInitTypeDef;
typedef struct { uint32_t Pin, Mode, Pull; } GPIO_InitTypeDef;
typedef struct { uint32_t Mode; } ADC_MultiModeTypeDef;
typedef struct {
    uint32_t SamplingTime, SingleDiff, OffsetNumber, Offset, Channel, Rank;
} ADC_ChannelConfTypeDef;
typedef struct {
    uint32_t ClockPrescaler, Resolution, DataAlign, GainCompensation;
    uint32_t ScanConvMode, EOCSelection, LowPowerAutoWait, ContinuousConvMode;
    uint32_t NbrOfConversion, DiscontinuousConvMode, ExternalTrigConv;
    uint32_t ExternalTrigConvEdge, SamplingMode, DMAContinuousRequests;
    uint32_t Overrun, OversamplingMode;
} ADC_InitTypeDef;
typedef struct { ADC_TypeDef *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
#define ENABLE 1
#define DISABLE 0
#define RCC_PERIPHCLK_ADC12 1
#define RCC_ADC12CLKSOURCE_SYSCLK 1
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_PIN_2 4
#define GPIO_PIN_3 8
#define GPIO_MODE_ANALOG 3
#define GPIO_NOPULL 0
#define ADC_CLOCK_ASYNC_DIV8 8
#define ADC_RESOLUTION_12B 12
#define ADC_DATAALIGN_RIGHT 0
#define ADC_SCAN_ENABLE 1
#define ADC_EOC_SINGLE_CONV 1
#define ADC_SOFTWARE_START 0
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0
#define ADC_SAMPLING_MODE_NORMAL 0
#define ADC_OVR_DATA_PRESERVED 0
#define ADC_MODE_INDEPENDENT 0
#define ADC_SINGLE_ENDED 0
#define ADC_SAMPLETIME_640CYCLES_5 640
#define ADC_OFFSET_NONE 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_4 4
#define ADC_REGULAR_RANK_1 1
#define ADC_REGULAR_RANK_2 2
#define ADC_REGULAR_RANK_3 3
#define ADC_REGULAR_RANK_4 4
#define ADC_REGULAR_RANK_5 5
#define ADC_REGULAR_RANK_6 6
#define ADC_REGULAR_RANK_7 7
#define ADC_REGULAR_RANK_8 8
#define ADC_REGULAR_RANK_9 9
#define ADC_FLAG_EOS 8U
#define ADC_FLAG_OVR 16U
#define __HAL_RCC_SYSCFG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_ADC12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *);
uint32_t HAL_RCC_GetSysClockFreq(void);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *, ADC_MultiModeTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
uint32_t HAL_ADC_GetError(const ADC_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_SYSCFG_DisableVREFBUF(void);
void HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t);
void HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t);
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void);
#endif
