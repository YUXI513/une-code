/* Compile this and the REAL Core/Src/buck_adc.c using a native C compiler:
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/adc_host -ICore/Inc
 *     tests/adc_host/test_buck_adc.c Core/Src/buck_adc.c -lm -o adc_test
 * Each scenario is a fresh process. This tests application behavior with
 * injected HAL faults; it does not simulate analog accuracy or STM32 timing. */
#include "buck_adc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

ADC_TypeDef test_adc;
VREFBUF_TypeDef test_vrefbuf;
static const char *scenario;
static uint32_t tick, rank, sequence, stopped;
static uint32_t plus = 1750U, minus = 1550U;
static uint32_t input_plus = 2000U, input_minus = 1600U;
static uint32_t channels[10];
#define IS(s) (strcmp(scenario, (s)) == 0)
#define UNUSED(x) ((void)(x))

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *p)
{ UNUSED(p); return IS("clock") ? HAL_ERROR : HAL_OK; }
uint32_t HAL_RCC_GetSysClockFreq(void) { return 168000000U; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *p) { UNUSED(port); UNUSED(p); }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h)
{ UNUSED(h); return IS("init") ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *h, ADC_MultiModeTypeDef *p)
{ UNUSED(h); UNUSED(p); return IS("multimode") ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t mode)
{ UNUSED(h); UNUSED(mode); return IS("calibration") ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *p)
{
    UNUSED(h);
    assert(p->Rank < 10U);
    channels[p->Rank] = p->Channel;
    return IS("channel") ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{
    UNUSED(h); rank = 0U; test_adc.ISR = 0U;
    ++sequence;
    return IS("start") ? HAL_BUSY : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h)
{ UNUSED(h); ++stopped; return IS("stop") ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout)
{
    UNUSED(h); UNUSED(timeout);
    if (IS("timeout")) return HAL_TIMEOUT;
    if (IS("conversion")) return HAL_ERROR;
    ++rank;
    test_adc.ISR = rank == 9U ? ADC_FLAG_EOS : 0U;
    if (IS("overrun")) test_adc.ISR |= ADC_FLAG_OVR;
    if (IS("sequence")) test_adc.ISR |= ADC_FLAG_EOS;
    if (IS("delay")) tick += 3U;
    if (IS("batch_delay") && rank == 1U) ++tick;
    if (IS("vref_loss_during") && sequence == BUCK_ADC_AVERAGE_COUNT && rank == 9U)
        test_vrefbuf.CSR &= ~VREFBUF_CSR_VRR;
    return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
    UNUSED(h);
    if (rank != 2U && rank != 4U && rank != 6U && rank != 8U)
        return 4095U; /* Dummy/discard conversions must never reach the average. */
    switch (channels[rank])
    {
        case ADC_CHANNEL_1:
            if (IS("input_rail") && sequence == 1U) return 0U;
            return input_plus + (IS("average") ? sequence - 1U : 0U);
        case ADC_CHANNEL_2: return input_minus;
        case ADC_CHANNEL_3:
            if (IS("fractional_negative")) return sequence == 1U ? 1599U : 1600U;
            if (IS("rail") && sequence == 1U) return 0U;
            return plus + (IS("average") ? 2U * (sequence - 1U) : 0U);
        case ADC_CHANNEL_4: return minus;
        default: assert(0); return 0U;
    }
}
uint32_t HAL_ADC_GetError(const ADC_HandleTypeDef *h) { UNUSED(h); return 0U; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void HAL_SYSCFG_DisableVREFBUF(void)
{
    test_vrefbuf.CSR &= ~(VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR);
}
void HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t mode)
{
    test_vrefbuf.CSR = (test_vrefbuf.CSR & ~VREFBUF_CSR_HIZ) | mode;
    if (IS("reference_mode")) test_vrefbuf.CSR |= VREFBUF_CSR_ENVR | VREFBUF_CSR_HIZ;
}
void HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t scale)
{
    test_vrefbuf.CSR = (test_vrefbuf.CSR & ~VREFBUF_CSR_VRS) | scale;
}
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void)
{
    test_vrefbuf.CSR |= VREFBUF_CSR_ENVR;
    if (IS("vref_timeout")) return HAL_TIMEOUT;
    if (IS("vref_enable_error")) return HAL_ERROR;
    if (!IS("vref_no_ready")) test_vrefbuf.CSR |= VREFBUF_CSR_VRR;
    return HAL_OK;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    assert(adc_reference_confirmed == 0U && adc_reference_voltage == BUCK_ADC_REFERENCE_VOLTS);
    assert(adc_sample_valid == 0U && isnan(buck_output_voltage));
    assert(adc_reference_hardware_confirmed == 0U);
    uint32_t init_reason = 0U;
    if (IS("reference_mode")) init_reason = BUCK_ADC_ERR_REFERENCE_MODE;
    if (IS("vref_timeout") || IS("vref_enable_error") || IS("vref_no_ready"))
        init_reason = BUCK_ADC_ERR_VREFBUF_READY;
    if (IS("vref_supply_low"))
    {
        adc_vdda_voltage = 2.7f;
        init_reason = BUCK_ADC_ERR_REFERENCE_MODE;
    }
    if (IS("clock")) init_reason = BUCK_ADC_ERR_CLOCK;
    if (IS("init") || IS("multimode")) init_reason = BUCK_ADC_ERR_INIT;
    if (IS("calibration")) init_reason = BUCK_ADC_ERR_CALIBRATION;
    if (IS("channel")) init_reason = BUCK_ADC_ERR_CHANNEL;
    HAL_StatusTypeDef status = BuckADC_Init();
    if (init_reason != 0U)
    {
        assert(status != HAL_OK && (adc_error_code & init_reason) != 0U);
        assert(BuckADC_ReadVoltage() != HAL_OK && adc_sample_valid == 0U);
        assert(isnan(BuckADC_GetOutputVoltage()));
        assert(isnan(BuckADC_GetOutputVoltageEstimate()));
        assert(adc_vrefbuf_ready == 0U || (init_reason &
               (BUCK_ADC_ERR_REFERENCE_MODE | BUCK_ADC_ERR_VREFBUF_READY)) == 0U);
        if (IS("vref_timeout") || IS("vref_enable_error") || IS("vref_no_ready"))
            assert((test_vrefbuf.CSR & VREFBUF_CSR_ENVR) == 0U);
        printf("PASS %s\n", scenario);
        return 0;
    }
    assert(status == HAL_OK && adc_clock_hz == 21000000U);
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
    assert((test_vrefbuf.CSR & (VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR))
           == (VREFBUF_CSR_ENVR | VREFBUF_CSR_VRR));
    assert((test_vrefbuf.CSR & VREFBUF_CSR_HIZ) == 0U && adc_vrefbuf_ready == 1U);
#else
    assert((test_vrefbuf.CSR & VREFBUF_CSR_ENVR) == 0U);
    assert((test_vrefbuf.CSR & VREFBUF_CSR_HIZ) != 0U);
#endif
    /* Synthetic test references, NOT confirmed voltages for the real board. */
    adc_reference_voltage = 2.5f;
    adc_vdda_voltage = 3.0f;
    adc_reference_confirmed = (IS("unknown") || IS("nominal_defaults")) ? 0U : 1U;
    adc_reference_hardware_confirmed = (IS("hardware_unknown") || IS("nominal_defaults")) ? 0U : 1U;
    if (IS("bad_reference")) adc_reference_voltage = 3.4f;
    if (IS("nan_reference")) adc_reference_voltage = NAN;
    if (IS("low_vdda"))
    {
        adc_reference_voltage = 2.2f;
        adc_vdda_voltage = 2.2f;
    }
    if (IS("negative") || IS("negative_nominal") || IS("both_negative_nominal")) plus = 1400U;
    if (IS("input_negative") || IS("both_negative_nominal")) input_plus = 1400U;
    if (IS("negative_nominal") || IS("both_negative_nominal"))
    {
        adc_reference_confirmed = 0U;
        adc_reference_hardware_confirmed = 0U;
    }
    if (IS("fractional_negative")) minus = 1600U;
    if (IS("wrap")) tick = UINT32_MAX - 10U;
    if (IS("vref_loss_before")) test_vrefbuf.CSR &= ~VREFBUF_CSR_VRR;
    status = BuckADC_ReadVoltage();
    uint32_t failure = 0U;
    if (IS("start")) failure = BUCK_ADC_ERR_START;
    if (IS("timeout")) failure = BUCK_ADC_ERR_TIMEOUT;
    if (IS("conversion")) failure = BUCK_ADC_ERR_CONVERSION;
    if (IS("overrun")) failure = BUCK_ADC_ERR_OVERRUN;
    if (IS("sequence")) failure = BUCK_ADC_ERR_SEQUENCE;
    if (IS("stop")) failure = BUCK_ADC_ERR_STOP;
    if (IS("delay") || IS("batch_delay")) failure = BUCK_ADC_ERR_STALE;
    if (IS("vref_loss_before") || IS("vref_loss_during")) failure = BUCK_ADC_ERR_VREFBUF_READY;
    if (failure != 0U)
    {
        assert(status != HAL_OK && (adc_error_code & failure) != 0U);
        assert(adc_raw_valid == 0U && adc_sample_valid == 0U);
        if (IS("vref_loss_before")) assert(stopped == 0U);
        else assert(stopped > 0U);
        assert(adc_input_sample_valid == 0U && adc_output_sample_valid == 0U);
        assert(isnan(buck_output_voltage) && isnan(BuckADC_GetOutputVoltage()));
        assert(isnan(buck_input_voltage) && isnan(BuckADC_GetInputVoltage()));
        assert(isnan(BuckADC_GetInputVoltageEstimate()) && isnan(BuckADC_GetOutputVoltageEstimate()));
        assert(isnan(BuckADC_GetInputVoltageForDisplay()) && isnan(BuckADC_GetOutputVoltageForDisplay()));
        if (IS("vref_loss_before") || IS("vref_loss_during"))
        {
            assert(adc_vrefbuf_ready == 0U && (test_vrefbuf.CSR & VREFBUF_CSR_ENVR) == 0U);
            assert(BuckADC_ReadVoltage() != HAL_OK);
        }
        if (IS("stop")) assert(BuckADC_ReadVoltage() != HAL_OK);
    }
    else
    {
        assert(status == HAL_OK && adc_raw_valid == 1U);
        if (IS("unknown") || IS("hardware_unknown") || IS("nominal_defaults"))
        {
            assert(adc_sample_valid == 0U && (adc_error_code & BUCK_ADC_ERR_REFERENCE));
            assert(isnan(BuckADC_GetInputVoltage()) && isnan(BuckADC_GetOutputVoltage()));
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
            assert(adc_input_estimate_valid == 1U && adc_output_estimate_valid == 1U);
            assert(fabsf(BuckADC_GetOutputVoltageEstimate() - 200.0f * 2.5f / 4095.0f * 40.0f) < 0.0001f);
            assert(fabsf(BuckADC_GetInputVoltageEstimate() - 400.0f * 2.5f / 4095.0f * 40.0f) < 0.0001f);
#else
            assert(isnan(BuckADC_GetOutputVoltageEstimate()));
#endif
        }
        else if (IS("bad_reference") || IS("nan_reference") || IS("low_vdda"))
        {
            assert(adc_sample_valid == 0U && (adc_error_code & BUCK_ADC_ERR_REFERENCE));
            assert(isnan(adc_voltage_diff) && isnan(buck_output_voltage));
            assert(isnan(adc_input_voltage_diff) && isnan(buck_input_voltage));
        }
        else if (IS("both_negative_nominal"))
        {
            assert(adc_sample_valid == 0U && adc_input_sample_valid == 0U && adc_output_sample_valid == 0U);
            assert(isnan(BuckADC_GetInputVoltage()) && isnan(BuckADC_GetOutputVoltage()));
            assert(isnan(BuckADC_GetInputVoltageEstimate()) && isnan(BuckADC_GetOutputVoltageEstimate()));
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
            assert(BuckADC_GetInputVoltageForDisplay() < 0.0f);
            assert(BuckADC_GetOutputVoltageForDisplay() < 0.0f);
#else
            assert(isnan(BuckADC_GetInputVoltageForDisplay()) && isnan(BuckADC_GetOutputVoltageForDisplay()));
#endif
            assert(adc_sample_valid == 0U); /* Diagnostic access never qualifies control feedback. */
        }
        else if (IS("negative_nominal") && !BUCK_ADC_INTERNAL_REFERENCE_TEST)
        {
            assert(adc_sample_valid == 0U);
            assert(isnan(BuckADC_GetInputVoltageForDisplay()) && isnan(BuckADC_GetOutputVoltageForDisplay()));
        }
        else if (IS("negative") || IS("fractional_negative") || IS("negative_nominal"))
        {
            assert(adc_sample_valid == 0U && (adc_error_code & BUCK_ADC_ERR_NEGATIVE));
            assert(adc_voltage_diff < 0.0f && buck_output_voltage < 0.0f);
            assert(isnan(BuckADC_GetOutputVoltage()));
            assert(isnan(BuckADC_GetOutputVoltageEstimate()));
            if (IS("negative_nominal"))
            {
                assert(adc_input_sample_valid == 0U && isnan(BuckADC_GetInputVoltage()));
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
                assert(BuckADC_GetInputVoltageForDisplay() > 0.0f);
                assert(BuckADC_GetOutputVoltageForDisplay() < 0.0f);
#else
                assert(isnan(BuckADC_GetOutputVoltageForDisplay()));
#endif
            }
            else
            {
                assert(adc_input_sample_valid == 1U && isfinite(BuckADC_GetInputVoltage()));
                assert(BuckADC_GetOutputVoltageForDisplay() == buck_output_voltage);
            }
            if (IS("fractional_negative"))
            {
                assert(adc_raw_plus == adc_raw_minus);
                assert(fabsf(buck_output_voltage + 2.5f / (4095.0f * 16.0f) * 40.0f) < 0.00001f);
            }
        }
        else if (IS("input_negative"))
        {
            assert(adc_sample_valid == 0U && adc_input_sample_valid == 0U);
            assert((adc_input_error_code & BUCK_ADC_ERR_NEGATIVE) != 0U);
            assert(buck_input_voltage < 0.0f && isnan(BuckADC_GetInputVoltage()));
            assert(adc_output_sample_valid == 1U && isfinite(BuckADC_GetOutputVoltage()));
            assert(BuckADC_GetInputVoltageForDisplay() == buck_input_voltage);
            assert(BuckADC_GetOutputVoltageForDisplay() > 0.0f);
        }
        else if (IS("rail"))
        {
            assert(adc_sample_valid == 0U && (adc_error_code & BUCK_ADC_ERR_RAIL));
            assert(adc_output_sample_valid == 0U && adc_input_sample_valid == 1U);
            assert(isnan(BuckADC_GetOutputVoltageForDisplay()));
        }
        else if (IS("input_rail"))
        {
            assert(adc_sample_valid == 0U && (adc_input_error_code & BUCK_ADC_ERR_RAIL));
            assert(adc_input_sample_valid == 0U && adc_output_sample_valid == 1U);
            assert(isnan(BuckADC_GetInputVoltageEstimate()));
            assert(isnan(BuckADC_GetInputVoltageForDisplay()));
        }
        else
        {
            assert(adc_sample_valid == 1U && adc_error_code == 0U);
            assert(BuckADC_GetRawPlus() == (IS("average") ? 1765U : plus));
            assert(BuckADC_GetRawMinus() == minus);
            assert(adc_input_raw_plus == (IS("average") ? 2008U : input_plus));
            assert(adc_input_raw_minus == input_minus);
            float expected = (float)(adc_raw_plus - adc_raw_minus) * 2.5f / 4095.0f * 40.0f;
            assert(fabsf(BuckADC_GetOutputVoltage() - expected) < 0.0001f);
            float average_input_plus = (float)input_plus + (IS("average") ? 7.5f : 0.0f);
            float expected_input = (average_input_plus - (float)input_minus) * 2.5f / 4095.0f * 40.0f;
            assert(fabsf(BuckADC_GetInputVoltage() - expected_input) < 0.0001f);
            if (IS("stale") || IS("wrap"))
            {
                tick += 31U;
                assert(isnan(BuckADC_GetOutputVoltage()));
                assert(isnan(BuckADC_GetInputVoltage()));
                assert(isnan(BuckADC_GetInputVoltageEstimate()) && isnan(BuckADC_GetOutputVoltageEstimate()));
                assert(isnan(BuckADC_GetInputVoltageForDisplay()) && isnan(BuckADC_GetOutputVoltageForDisplay()));
                assert(adc_sample_valid == 0U && (adc_error_code & BUCK_ADC_ERR_STALE));
            }
            if (IS("revoke_reference"))
            {
                adc_reference_confirmed = 0U;
                assert(isnan(BuckADC_GetOutputVoltage()) && adc_sample_valid == 0U);
            }
            if (IS("revoke_hardware"))
            {
                adc_reference_hardware_confirmed = 0U;
                assert(isnan(BuckADC_GetOutputVoltage()) && adc_sample_valid == 0U);
            }
            if (IS("vref_loss_getter"))
            {
                test_vrefbuf.CSR &= ~VREFBUF_CSR_VRR;
                assert(isnan(BuckADC_GetOutputVoltage()));
                assert(isnan(BuckADC_GetInputVoltageEstimate()) && isnan(BuckADC_GetOutputVoltageEstimate()));
                assert((adc_error_code & BUCK_ADC_ERR_VREFBUF_READY) != 0U);
                assert(adc_vrefbuf_ready == 0U && (test_vrefbuf.CSR & VREFBUF_CSR_ENVR) == 0U);
            }
        }
    }
    printf("PASS %s\n", scenario);
    return 0;
}
