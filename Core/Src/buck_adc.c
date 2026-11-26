
#include "buck_adc.h"
#include <math.h>

#define ADC_POLL_TIMEOUT_MS  2U
#define ADC_SEQUENCE_RANKS   9U
#define ADC_MAX_RAW          4095U

volatile uint32_t adc_raw_plus, adc_raw_minus;
volatile uint32_t adc_input_raw_plus, adc_input_raw_minus;
volatile float adc_voltage_plus = NAN, adc_voltage_minus = NAN;
volatile float adc_voltage_diff = NAN, buck_output_voltage = NAN;
volatile float adc_input_voltage_plus = NAN, adc_input_voltage_minus = NAN;
volatile float adc_input_voltage_diff = NAN, buck_input_voltage = NAN;
volatile uint32_t adc_raw_valid, adc_sample_valid;
volatile uint32_t adc_error_code = BUCK_ADC_ERR_REFERENCE;
volatile uint32_t adc_input_sample_valid, adc_output_sample_valid;
volatile uint32_t adc_input_estimate_valid, adc_output_estimate_valid;
volatile uint32_t adc_input_error_code = BUCK_ADC_ERR_REFERENCE;
volatile uint32_t adc_output_error_code = BUCK_ADC_ERR_REFERENCE;
volatile uint32_t adc_hal_error, adc_last_hal_status, adc_failure_count;
volatile uint32_t adc_update_count, adc_last_sample_ms;
volatile uint32_t adc_clock_hz, adc_vrefbuf_csr;
volatile uint32_t adc_vrefbuf_ready, adc_vrefbuf_fault_csr;
volatile uint32_t adc_vrefbuf_startup_status = HAL_ERROR;
volatile float adc_reference_voltage = BUCK_ADC_REFERENCE_VOLTS;
volatile float adc_vdda_voltage = BUCK_ADC_VDDA_VOLTS;
volatile uint32_t adc_reference_confirmed = BUCK_ADC_REFERENCE_CONFIRMED;
volatile uint32_t adc_reference_hardware_confirmed = BUCK_ADC_REFERENCE_HARDWARE_CONFIRMED;

static ADC_HandleTypeDef hadc1;
static uint32_t adc_initialized;
static uint32_t init_error;

static void InvalidateVoltages(void)
{
    adc_sample_valid = 0U;
    adc_input_sample_valid = 0U;
    adc_output_sample_valid = 0U;
    adc_input_estimate_valid = 0U;
    adc_output_estimate_valid = 0U;
    adc_voltage_plus = NAN;
    adc_voltage_minus = NAN;
    adc_voltage_diff = NAN;
    buck_output_voltage = NAN;
    adc_input_voltage_plus = NAN;
    adc_input_voltage_minus = NAN;
    adc_input_voltage_diff = NAN;
    buck_input_voltage = NAN;
}

static HAL_StatusTypeDef Fail(uint32_t reason, HAL_StatusTypeDef status)
{
    adc_raw_valid = 0U;
    InvalidateVoltages();
    adc_error_code |= reason;
    adc_input_error_code |= adc_error_code;
    adc_output_error_code |= adc_error_code;
    adc_last_hal_status = (uint32_t)status;
    adc_hal_error = HAL_ADC_GetError(&hadc1);
    ++adc_failure_count;
    return status;
}

static uint32_t ReferenceScaleUsable(void)
{
    float ref = adc_reference_voltage;
    float supply = adc_vdda_voltage;
    /* This board plans 3.3 V VDDA. Below 2.4 V the analog input switch
     * booster needs separate configuration, which we do not guess/change.
     * Require a confirmed supply in the supported range before conversion. */
    float minimum_supply = BUCK_ADC_INTERNAL_REFERENCE_TEST ? 2.8f : 2.4f;
    return isfinite(ref) && isfinite(supply) && supply >= minimum_supply
        && supply <= 3.6f && ref >= 2.0f && ref <= supply;
}

static uint32_t ReferenceValid(void)
{
    return adc_reference_confirmed == 1U && ReferenceScaleUsable()
        && (!BUCK_ADC_INTERNAL_REFERENCE_TEST || adc_reference_hardware_confirmed == 1U);
}

static uint32_t ReferenceModeHealthy(void)
{
    adc_vrefbuf_csr = VREFBUF->CSR;
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
    uint32_t mask = VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ | VREFBUF_CSR_VRR | VREFBUF_CSR_VRS;
    return (adc_vrefbuf_csr & mask)
        == (VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR | SYSCFG_VREFBUF_VOLTAGE_SCALE1);
#else
    return (adc_vrefbuf_csr & (VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ)) == VREFBUF_CSR_HIZ;
#endif
}

static HAL_StatusTypeDef ReferenceFault(uint32_t reason, HAL_StatusTypeDef status)
{
    adc_vrefbuf_fault_csr = VREFBUF->CSR;
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
    HAL_SYSCFG_DisableVREFBUF();
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);
#endif
    adc_vrefbuf_csr = VREFBUF->CSR;
    adc_vrefbuf_ready = 0U;
    adc_initialized = 0U;
    init_error = reason;
    return Fail(reason, status);
}

static HAL_StatusTypeDef InitializeReference(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    adc_vrefbuf_ready = 0U;
    HAL_SYSCFG_DisableVREFBUF();
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
    if (!isfinite(adc_vdda_voltage) || adc_vdda_voltage < 2.8f || adc_vdda_voltage > 3.6f)
        return ReferenceFault(BUCK_ADC_ERR_REFERENCE_MODE, HAL_ERROR);

    HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
    /* Installed HAL waits for VRR, bounded by its 10 ms timeout. */
    HAL_StatusTypeDef status = HAL_SYSCFG_EnableVREFBUF();
    adc_vrefbuf_startup_status = (uint32_t)status;
    if (status != HAL_OK)
        return ReferenceFault(BUCK_ADC_ERR_VREFBUF_READY, status);
    if (!ReferenceModeHealthy())
    {
        adc_vrefbuf_startup_status = HAL_ERROR;
        uint32_t reason = (adc_vrefbuf_csr & VREFBUF_CSR_VRR) == 0U
            ? BUCK_ADC_ERR_VREFBUF_READY : BUCK_ADC_ERR_REFERENCE_MODE;
        return ReferenceFault(reason, HAL_ERROR);
    }
    /* Bench startup only; VRR does not certify the unknown capacitor. */
    HAL_Delay(1U);
    if (!ReferenceModeHealthy())
        return ReferenceFault(BUCK_ADC_ERR_VREFBUF_READY, HAL_ERROR);
    adc_vrefbuf_ready = 1U;
#else
    if (!ReferenceModeHealthy())
        return ReferenceFault(BUCK_ADC_ERR_REFERENCE_MODE, HAL_ERROR);
    adc_vrefbuf_startup_status = HAL_OK;
#endif
    return HAL_OK;
}

HAL_StatusTypeDef BuckADC_Init(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};
    GPIO_InitTypeDef gpio = {0};
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef channel = {0};
    static const uint32_t ranks[ADC_SEQUENCE_RANKS] = {
        ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3,
        ADC_REGULAR_RANK_4, ADC_REGULAR_RANK_5, ADC_REGULAR_RANK_6,
        ADC_REGULAR_RANK_7, ADC_REGULAR_RANK_8, ADC_REGULAR_RANK_9
    };
    static const uint32_t channels[ADC_SEQUENCE_RANKS] = {
        ADC_CHANNEL_1, ADC_CHANNEL_1, ADC_CHANNEL_1,
        ADC_CHANNEL_2, ADC_CHANNEL_2,
        ADC_CHANNEL_3, ADC_CHANNEL_3,
        ADC_CHANNEL_4, ADC_CHANNEL_4
    };
    HAL_StatusTypeDef status;

    /* One owner/one initialization: never reset the shared ADC12 block. */
    if (adc_initialized != 0U)
    {
        return HAL_OK;
    }
    adc_error_code = ReferenceValid() ? 0U : BUCK_ADC_ERR_REFERENCE;
    adc_input_error_code = adc_error_code;
    adc_output_error_code = adc_error_code;
    adc_raw_valid = 0U;
    InvalidateVoltages();
    hadc1.Instance = ADC1;

    status = InitializeReference();
    if (status != HAL_OK) return status;
    clock.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    clock.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    status = HAL_RCCEx_PeriphCLKConfig(&clock);
    if (status != HAL_OK)
    {
        init_error = BUCK_ADC_ERR_CLOCK;
        return Fail(init_error, status);
    }
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    adc_clock_hz = HAL_RCC_GetSysClockFreq() / 8U; /* 168 MHz / 8 = 21 MHz */
    if (adc_clock_hz == 0U || adc_clock_hz > 42000000U)
    {
        init_error = BUCK_ADC_ERR_CLOCK;
        return Fail(init_error, HAL_ERROR);
    }
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0U;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    /* DR reads release the next rank, so polling cannot lose U+ to U-. */
    hadc1.Init.LowPowerAutoWait = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = ADC_SEQUENCE_RANKS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    /* The installed G4 HAL exits deep-power-down, enables the ADC internal
     * regulator and waits tADCVREG_STUP here. This is NOT VREFBUF. */
    status = HAL_ADC_Init(&hadc1);
    if (status != HAL_OK)
    {
        init_error = BUCK_ADC_ERR_INIT;
        return Fail(init_error, status);
    }
    multimode.Mode = ADC_MODE_INDEPENDENT;
    status = HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
    if (status != HAL_OK)
    {
        init_error = BUCK_ADC_ERR_INIT;
        return Fail(init_error, status);
    }
    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        init_error = BUCK_ADC_ERR_CALIBRATION;
        return Fail(init_error, status);
    }
    channel.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    /* ES0430 2.7.8/2.7.9: after a >1 ms idle gap discard the first
     * conversion, and avoid keeping the conversion that switches channel.
     * Sequence: IN+(dummy), IN+(keep), IN+(discard), IN-(keep),
     * IN-(discard), OUT+(keep), OUT+(discard), OUT-(keep), OUT-(discard).
     * Handles all silicon revisions without changing TIM1 or using DMA. */
    for (uint32_t rank = 0U; rank < ADC_SEQUENCE_RANKS; ++rank)
    {
        channel.Channel = channels[rank];
        channel.Rank = ranks[rank];
        status = HAL_ADC_ConfigChannel(&hadc1, &channel);
        if (status != HAL_OK)
        {
            init_error = BUCK_ADC_ERR_CHANNEL;
            return Fail(init_error, status);
        }
    }
    /* Exceeds the calibration-to-enable minimum (4 ADC cycles) at 21 MHz.
     * Startup only, in main context; SysTick must already be running. */
    HAL_Delay(1U);
    init_error = 0U;
    adc_initialized = 1U;
    adc_last_hal_status = HAL_OK;
    return HAL_OK;
}

static HAL_StatusTypeDef ReadFrame(uint32_t values[4], uint32_t *reason)
{
    uint32_t started = HAL_GetTick();
    HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
        *reason = BUCK_ADC_ERR_START;
    }
    for (uint32_t rank = 0U; status == HAL_OK && rank < ADC_SEQUENCE_RANKS; ++rank)
    {
        status = HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            *reason = status == HAL_TIMEOUT ? BUCK_ADC_ERR_TIMEOUT
                                           : BUCK_ADC_ERR_CONVERSION;
            break;
        }
        uint32_t flags = hadc1.Instance->ISR;
        if ((flags & ADC_FLAG_OVR) != 0U)
        {
            *reason = BUCK_ADC_ERR_OVERRUN;
            status = HAL_ERROR;
            break;
        }
        /* AutoWait leaves EOS/EOC set until the DR read. Verify rank order. */
        if (((flags & ADC_FLAG_EOS) != 0U) != (rank == ADC_SEQUENCE_RANKS - 1U))
        {
            *reason = BUCK_ADC_ERR_SEQUENCE;
            status = HAL_ERROR;
            break;
        }
        uint32_t value = HAL_ADC_GetValue(&hadc1);
        /* Keep ranks 2/4/6/8, in PA0/PA1/PA2/PA3 order. */
        if ((rank & 1U) != 0U) values[(rank - 1U) / 2U] = value;
        if (value > ADC_MAX_RAW)
        {
            *reason = BUCK_ADC_ERR_CONVERSION;
            status = HAL_ERROR;
            break;
        }
    }
    /* Always stop/disable, including on errors. A failed stop latches the
     * module unusable until explicit reinitialization/reset, avoiding an
     * untrusted sequencer restart. */
    HAL_StatusTypeDef stopped = HAL_ADC_Stop(&hadc1);
    if (stopped != HAL_OK)
    {
        *reason |= BUCK_ADC_ERR_STOP;
        adc_initialized = 0U;
        init_error = BUCK_ADC_ERR_STOP;
        status = stopped;
    }
    if (status == HAL_OK && (uint32_t)(HAL_GetTick() - started) >= 2U)
    {
        *reason = BUCK_ADC_ERR_STALE; /* Reject a debugger-stalled sequence. */
        status = HAL_TIMEOUT;
    }
    return status;
}

HAL_StatusTypeDef BuckADC_ReadVoltage(void)
{
    uint32_t batch_started = HAL_GetTick();
    uint32_t sums[4] = {0U};
    uint32_t reason = 0U, input_rail = 0U, output_rail = 0U;
    adc_raw_valid = 0U;
    InvalidateVoltages();
    adc_error_code = ReferenceValid() ? 0U : BUCK_ADC_ERR_REFERENCE;
    adc_input_error_code = adc_error_code;
    adc_output_error_code = adc_error_code;
    if (adc_initialized == 0U)
    {
        return Fail(init_error | BUCK_ADC_ERR_INIT, HAL_ERROR);
    }
    if (!ReferenceModeHealthy())
        return ReferenceFault(BUCK_ADC_ERR_VREFBUF_READY, HAL_ERROR);
    for (uint32_t sample = 0U; sample < BUCK_ADC_AVERAGE_COUNT; ++sample)
    {
        uint32_t values[4] = {0U};
        HAL_StatusTypeDef status = ReadFrame(values, &reason);
        if (status != HAL_OK) return Fail(reason, status);
        /* Check individual readings: averaging must not hide clipped data. */
        for (uint32_t index = 0U; index < 4U; ++index)
        {
            if (values[index] == 0U || values[index] == ADC_MAX_RAW)
            {
                if (index < 2U) input_rail = BUCK_ADC_ERR_RAIL;
                else output_rail = BUCK_ADC_ERR_RAIL;
            }
            sums[index] += values[index];
        }
    }
    if ((uint32_t)(HAL_GetTick() - batch_started) >= BUCK_ADC_UPDATE_MS)
        return Fail(BUCK_ADC_ERR_STALE, HAL_TIMEOUT);
    if (!ReferenceModeHealthy())
        return ReferenceFault(BUCK_ADC_ERR_VREFBUF_READY, HAL_ERROR);
    adc_input_raw_plus = (sums[0] + BUCK_ADC_AVERAGE_COUNT / 2U) / BUCK_ADC_AVERAGE_COUNT;
    adc_input_raw_minus = (sums[1] + BUCK_ADC_AVERAGE_COUNT / 2U) / BUCK_ADC_AVERAGE_COUNT;
    adc_raw_plus = (sums[2] + BUCK_ADC_AVERAGE_COUNT / 2U) / BUCK_ADC_AVERAGE_COUNT;
    adc_raw_minus = (sums[3] + BUCK_ADC_AVERAGE_COUNT / 2U) / BUCK_ADC_AVERAGE_COUNT;
    adc_raw_valid = 1U;
    adc_last_sample_ms = HAL_GetTick();
    ++adc_update_count;
    adc_last_hal_status = HAL_OK;
    adc_hal_error = HAL_ADC_GetError(&hadc1);
    adc_vrefbuf_csr = VREFBUF->CSR;
    adc_input_error_code |= input_rail;
    adc_output_error_code |= output_rail;
    /* Raw signed ordering is meaningful even before VREF+ is known. */
    if (sums[0] < sums[1]) adc_input_error_code |= BUCK_ADC_ERR_NEGATIVE;
    if (sums[2] < sums[3]) adc_output_error_code |= BUCK_ADC_ERR_NEGATIVE;
    /* Explicit bench mode may calculate nominal estimates. Their separate
     * flags/getters do not turn an unconfirmed measurement into PI data. */
    if (ReferenceValid() || (BUCK_ADC_INTERNAL_REFERENCE_TEST && ReferenceScaleUsable()))
    {
        /* Keep sub-count information from the average. Integer raw Watch
         * values above are rounded for inspection only, not conversion. */
        float scale = adc_reference_voltage / ((float)ADC_MAX_RAW * BUCK_ADC_AVERAGE_COUNT);
        adc_voltage_plus = (float)sums[2] * scale;
        adc_voltage_minus = (float)sums[3] * scale;
        adc_voltage_diff = ((float)sums[2] - (float)sums[3]) * scale;
        buck_output_voltage = adc_voltage_diff * BUCK_ADC_OUTPUT_DIVIDER_RATIO;
        adc_input_voltage_plus = (float)sums[0] * scale;
        adc_input_voltage_minus = (float)sums[1] * scale;
        adc_input_voltage_diff = ((float)sums[0] - (float)sums[1]) * scale;
        buck_input_voltage = adc_input_voltage_diff * BUCK_ADC_INPUT_DIVIDER_RATIO;
    }
    adc_input_sample_valid = adc_input_error_code == 0U ? 1U : 0U;
    adc_output_sample_valid = adc_output_error_code == 0U ? 1U : 0U;
    adc_input_estimate_valid = isfinite(buck_input_voltage)
        && (adc_input_error_code & ~BUCK_ADC_ERR_REFERENCE) == 0U;
    adc_output_estimate_valid = isfinite(buck_output_voltage)
        && (adc_output_error_code & ~BUCK_ADC_ERR_REFERENCE) == 0U;
    adc_error_code = adc_input_error_code | adc_output_error_code;
    adc_sample_valid = adc_error_code == 0U ? 1U : 0U;
    return HAL_OK;
}

uint32_t BuckADC_GetRawPlus(void) { return adc_raw_plus; }
uint32_t BuckADC_GetRawMinus(void) { return adc_raw_minus; }

static void CheckVoltageFreshness(void)
{
    if (adc_initialized != 0U && !ReferenceModeHealthy())
        (void)ReferenceFault(BUCK_ADC_ERR_VREFBUF_READY, HAL_ERROR);
    if (!ReferenceValid())
    {
        adc_sample_valid = 0U;
        adc_input_sample_valid = 0U;
        adc_output_sample_valid = 0U;
        adc_error_code |= BUCK_ADC_ERR_REFERENCE;
        adc_input_error_code |= BUCK_ADC_ERR_REFERENCE;
        adc_output_error_code |= BUCK_ADC_ERR_REFERENCE;
    }
    if (!ReferenceScaleUsable())
    {
        adc_input_estimate_valid = 0U;
        adc_output_estimate_valid = 0U;
    }
    if ((uint32_t)(HAL_GetTick() - adc_last_sample_ms) > 3U * BUCK_ADC_UPDATE_MS)
    {
        adc_sample_valid = 0U;
        adc_input_sample_valid = 0U;
        adc_output_sample_valid = 0U;
        adc_input_estimate_valid = 0U;
        adc_output_estimate_valid = 0U;
        adc_error_code |= BUCK_ADC_ERR_STALE;
        adc_input_error_code |= BUCK_ADC_ERR_STALE;
        adc_output_error_code |= BUCK_ADC_ERR_STALE;
    }
}

/* Watch keeps the signed result on anomalies; getters intended for control
 * return NAN for EVERY invalid/stale result. No negative result is clamped. */
float BuckADC_GetDifferentialVoltage(void)
{
    CheckVoltageFreshness();
    return adc_output_sample_valid ? adc_voltage_diff : NAN;
}

float BuckADC_GetOutputVoltage(void)
{
    CheckVoltageFreshness();
    return adc_output_sample_valid ? buck_output_voltage : NAN;
}

float BuckADC_GetInputVoltage(void)
{
    CheckVoltageFreshness();
    return adc_input_sample_valid ? buck_input_voltage : NAN;
}

float BuckADC_GetInputVoltageEstimate(void)
{
    CheckVoltageFreshness();
    return adc_input_estimate_valid ? buck_input_voltage : NAN;
}

float BuckADC_GetOutputVoltageEstimate(void)
{
    CheckVoltageFreshness();
    return adc_output_estimate_valid ? buck_output_voltage : NAN;
}

static uint32_t SignedEstimateUsable(uint32_t error)
{
    uint32_t diagnostic_flags = BUCK_ADC_ERR_NEGATIVE;
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
    diagnostic_flags |= BUCK_ADC_ERR_REFERENCE;
#endif
    return adc_raw_valid != 0U && ReferenceScaleUsable()
        && (ReferenceValid() || BUCK_ADC_INTERNAL_REFERENCE_TEST)
        && (error & ~diagnostic_flags) == 0U;
}

float BuckADC_GetInputVoltageForDisplay(void)
{
    CheckVoltageFreshness();
    return SignedEstimateUsable(adc_input_error_code) ? buck_input_voltage : NAN;
}

float BuckADC_GetOutputVoltageForDisplay(void)
{
    CheckVoltageFreshness();
    return SignedEstimateUsable(adc_output_error_code) ? buck_output_voltage : NAN;
}

float BuckADC_GetOutputVoltageStartupEstimate(void)
{
    CheckVoltageFreshness();
    /* Same reference/freshness/error gates, explicit startup caller only.
     * Normal validity/estimate flags and signed raw values stay unchanged. */
    return SignedEstimateUsable(adc_output_error_code) ? buck_output_voltage : NAN;
}
