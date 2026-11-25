/* Native tests of the real automatic-start adapter with ADC/PWM doubles.
 * They validate sequencing/interlocks, not analog accuracy or LC stability. */
#include "buck_control.h"
#include "buck_adc.h"
#include "buck_pwm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile uint32_t adc_raw_valid = 1U, adc_last_sample_ms = 100U, adc_update_count = 1U;
static uint32_t tick = 100U, running, starts, reject_duty;
static uint32_t other_adc_error;
static float vin = 20.0f, vout = 0.1f;
uint32_t HAL_GetTick(void) { return tick; }
float BuckADC_GetInputVoltage(void) { return NAN; }
float BuckADC_GetOutputVoltage(void) { return NAN; }
float BuckADC_GetInputVoltageEstimate(void) { return adc_raw_valid ? vin : NAN; }
float BuckADC_GetOutputVoltageEstimate(void)
{
    /* The real ADC rejects negative differential for feedback. */
    return adc_raw_valid && !other_adc_error && vout >= 0.0f ? vout : NAN;
}
float BuckADC_GetOutputVoltageStartupEstimate(void)
{
    return adc_raw_valid && !other_adc_error ? vout : NAN;
}
void BuckPWM_Stop(void) { running = 0U; }
void BuckPWM_Start(void) { running = 1U; ++starts; }
uint32_t BuckPWM_IsRunning(void) { return running; }
uint32_t BuckPWM_SetDuty(float duty)
{
    assert(isfinite(duty) && duty >= 0.08f && duty <= 0.80f);
    return !reject_duty;
}
static void Sample(void)
{
    tick += 10U;
    adc_last_sample_ms = tick;
    ++adc_update_count;
    BuckControl_Process(tick);
}
static void WaitSamples(uint32_t count)
{
    for (uint32_t i = 0U; i < count; ++i) Sample();
}
static void QualifyAndStart(void)
{
    uint32_t before = starts;
    WaitSamples(100U);
    assert(starts == before && running == 0U);
    Sample();
    assert(starts == before + 1U && running == 1U);
    assert(buck_control_state == BUCK_CONTROL_SOFT_START);
    assert(buck_control_auto_pending == 0U);
    /* Automatic software permission is not a fake hardware confirmation. */
    assert(buck_control_hardware_ready == 0U);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *s = argv[1];
#define IS(name) (strcmp(s, name) == 0)
    if (IS("wrap")) tick = adc_last_sample_ms = UINT32_MAX - 500U;
    if (IS("startup_timeout_wrap")) tick = adc_last_sample_ms = UINT32_MAX - 1100U;
    BuckControl_Init();
    assert(starts == 0U && running == 0U);
    assert(buck_control_state == BUCK_CONTROL_WAIT_START && buck_control_auto_pending == 1U);
    assert(buck_control_hardware_ready == 0U);
    /* Repeated processing of the startup frame must never start PWM. */
    for (uint32_t i = 0U; i < 1000U; ++i) BuckControl_Process(tick);
    assert(buck_control_auto_samples == 0U && starts == 0U);

    if (IS("wait_adc") || IS("negative") || IS("wait_vin") || IS("wait_vout"))
    {
        if (IS("wait_adc")) adc_raw_valid = 0U;
        if (IS("negative")) vout = -0.501f;
        if (IS("wait_vin")) vin = 14.9f;
        if (IS("wait_vout")) vout = 5.0f;
        WaitSamples(200U);
        assert(starts == 0U && buck_control_auto_samples == 0U);
        assert(buck_control_state == BUCK_CONTROL_WAIT_START);
        uint32_t reason = IS("wait_vin") ? BUCK_CONTROL_WAIT_VIN
            : IS("wait_vout") ? BUCK_CONTROL_WAIT_VOUT : BUCK_CONTROL_WAIT_ADC;
        assert(buck_control_auto_wait == reason);
        adc_raw_valid = 1U; vin = 20.0f; vout = 0.1f;
        QualifyAndStart();
    }
    else if (IS("intermittent"))
    {
        for (uint32_t i = 0U; i < 300U; ++i)
        {
            vout = i % 50U == 49U ? -0.501f : 0.01f;
            Sample();
        }
        assert(starts == 0U && running == 0U);
        vout = 0.1f;
        QualifyAndStart();
    }
    else if (IS("stale"))
    {
        WaitSamples(50U);
        tick += 31U;
        BuckControl_Process(tick);
        assert(buck_control_auto_samples == 0U && starts == 0U);
        QualifyAndStart();
    }
    else if (IS("no_new_frame"))
    {
        Sample();
        for (uint32_t i = 0U; i < 200U; ++i)
        {
            tick += 10U;
            BuckControl_Process(tick);
        }
        assert(starts == 0U && buck_control_auto_samples == 0U);
        QualifyAndStart();
    }
    else if (IS("stop_wait") || IS("clear_wait") || IS("preview_wait"))
    {
        WaitSamples(50U);
        buck_control_request = IS("stop_wait") ? BUCK_CONTROL_CMD_STOP
            : IS("clear_wait") ? BUCK_CONTROL_CMD_CLEAR_FAULT : BUCK_CONTROL_CMD_PREVIEW;
        BuckControl_Process(tick);
        WaitSamples(200U);
        assert(starts == 0U && buck_control_auto_pending == 0U);
        puts("PASS operator command cancels automatic startup until reset");
        return 0;
    }
    else if (IS("bad_pwm"))
    {
        reject_duty = 1U;
        WaitSamples(101U);
        assert(starts == 0U && running == 0U);
        assert(buck_control_state == BUCK_CONTROL_FAULT);
        assert(buck_control_fault & BUCK_CONTROL_ERR_PWM);
        reject_duty = 0U;
    }
    else if (IS("negative_startup_ok") || IS("negative_limit") || IS("startup_timeout")
             || IS("startup_other_adc_error") || IS("startup_timeout_wrap")
             || IS("startup_drop_below_limit"))
    {
        vout = IS("negative_limit") ? -0.50f : -0.45f;
        if (IS("startup_other_adc_error"))
        {
            other_adc_error = 1U;
            WaitSamples(200U);
            assert(starts == 0U && running == 0U && buck_control_auto_samples == 0U);
            assert(buck_control_auto_wait == BUCK_CONTROL_WAIT_ADC);
            puts("PASS startup exception cannot bypass other ADC faults");
            return 0;
        }
        QualifyAndStart();
        assert(buck_control_startup_negative == 1U);
        assert(buck_control_vout == vout); /* No zero clamp or hidden calibration. */
        if (IS("startup_drop_below_limit"))
        {
            vout = -0.501f;
            Sample();
            assert(running == 0U && (buck_control_fault & BUCK_CONTROL_ERR_ADC));
        }
        else if (IS("startup_timeout") || IS("startup_timeout_wrap"))
        {
            WaitSamples(19U);
            assert(running == 1U);
            Sample(); /* Exactly 200 ms: signed startup access has expired. */
            assert(buck_control_fault & BUCK_CONTROL_ERR_STARTUP_VOUT);
            assert(strcmp(BuckControl_StateText(), "START VOUT LOW - OFF") == 0);
        }
        else
        {
            vout = 0.60f;
            Sample();
            assert(running == 1U && buck_control_startup_negative == 0U);
            vout = 10.0f;
            WaitSamples(220U);
            assert(running == 1U && buck_control_state == BUCK_CONTROL_RUNNING);
            puts("PASS bounded signed startup reaches normal control without clamping");
            return 0;
        }
    }
    else
    {
        QualifyAndStart();
        if (IS("adc_loss")) { adc_raw_valid = 0U; Sample(); }
        else if (IS("negative_running"))
        {
            vout = 0.60f; Sample(); /* Close the exception before 200 ms. */
            assert(running == 1U);
            vout = -0.02f; Sample();
        }
        else if (IS("overvoltage")) { vout = 11.1f; Sample(); }
        else if (IS("watchdog"))
        {
            tick += 31U;
            BuckControl_TickGuard(tick);
            BuckControl_Process(tick);
        }
        else if (IS("stop_running"))
        {
            buck_control_request = BUCK_CONTROL_CMD_STOP;
            BuckControl_Process(tick);
            WaitSamples(200U);
            assert(running == 0U && starts == 1U && buck_control_state == BUCK_CONTROL_STOPPED);
            puts("PASS stop prevents automatic restart");
            return 0;
        }
        else
        {
            assert(IS("normal") || IS("wrap"));
            vout = 10.0f;
            WaitSamples(220U);
            assert(running == 1U && starts == 1U && buck_control_state == BUCK_CONTROL_RUNNING);
            puts("PASS qualified automatic start, soft-start and continuous running");
            return 0;
        }
    }
    if (IS("wait_adc") || IS("negative") || IS("wait_vin") || IS("wait_vout")
        || IS("intermittent") || IS("stale") || IS("no_new_frame"))
    {
        assert(running == 1U && starts == 1U);
        puts("PASS blocked startup requires a new full qualification window");
        return 0;
    }
    assert(buck_control_state == BUCK_CONTROL_FAULT && running == 0U);
    uint32_t previous_starts = starts;
    adc_raw_valid = 1U; vout = 0.1f;
    WaitSamples(200U);
    assert(starts == previous_starts && running == 0U);
    buck_control_request = BUCK_CONTROL_CMD_CLEAR_FAULT;
    BuckControl_Process(tick);
    WaitSamples(200U);
    assert(starts == previous_starts && running == 0U);
    assert(buck_control_auto_pending == 0U && buck_control_hardware_ready == 0U);
    puts("PASS fault latches and clear never silently rearms automatic startup");
    return 0;
}
