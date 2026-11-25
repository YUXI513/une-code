#include "buck_control.h"
#include "buck_adc.h"
#include "buck_pwm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile uint32_t adc_raw_valid = 1U, adc_last_sample_ms = 100U, adc_update_count = 1U;
static uint32_t tick = 100U, running, starts, stops, reject_duty, inject_watchdog;
static float vin = 20.0f, vout = 0.1f;
uint32_t HAL_GetTick(void) { return tick; }
/* Deliberately unconfirmed: only explicit bench/preview paths accept them. */
float BuckADC_GetInputVoltage(void) { return NAN; }
float BuckADC_GetOutputVoltage(void) { return NAN; }
float BuckADC_GetInputVoltageEstimate(void) { return adc_raw_valid ? vin : NAN; }
float BuckADC_GetOutputVoltageEstimate(void) { return adc_raw_valid ? vout : NAN; }
float BuckADC_GetOutputVoltageStartupEstimate(void)
{
    assert(0); /* Default/manual firmware must never take this exception. */
    return NAN;
}
void BuckPWM_Stop(void) { running = 0U; ++stops; }
void BuckPWM_Start(void) { running = 1U; ++starts; }
uint32_t BuckPWM_IsRunning(void) { return running; }
uint32_t BuckPWM_SetDuty(float duty)
{
    assert(isfinite(duty) && duty >= 0.08f && duty <= 0.80f);
    if (inject_watchdog)
    {
        tick += 31U;
        BuckControl_TickGuard(tick);
    }
    return !reject_duty;
}
static void Sample(void)
{
    tick += 10U;
    adc_last_sample_ms = tick;
    ++adc_update_count;
    BuckControl_Process(tick);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *s = argv[1];
    if (strcmp(s, "wrap") == 0)
    {
        tick = UINT32_MAX - 15U;
        adc_last_sample_ms = tick;
    }
    BuckControl_Init();
    assert(starts == 0U && running == 0U && buck_control_hardware_ready == 0U);
    assert(buck_control_state == BUCK_CONTROL_PREVIEW);
    Sample();
    assert(starts == 0U && buck_control_pwm_active == 0U && buck_control_steps == 1U);
    assert(buck_control_feedback_estimate == 1U);
    BuckControl_Process(tick);
    assert(buck_control_steps == 1U); /* No reuse of one ADC frame. */
    if (strcmp(s, "preview") == 0) { puts("PASS preview never starts PWM"); return 0; }
    if (strcmp(s, "inhibit") == 0)
    {
        buck_control_request = BUCK_CONTROL_CMD_START;
        BuckControl_Process(tick);
        assert(starts == 0U && buck_control_state == BUCK_CONTROL_INHIBITED);
        puts("PASS default/interlock inhibits power start"); return 0;
    }
#if BUCK_CONTROL_POWER_STAGE
    buck_control_hardware_ready = 1U;
    if (strcmp(s, "prebias") == 0) vout = 5.0f;
    if (strcmp(s, "bad_pwm") == 0) reject_duty = 1U;
    if (strcmp(s, "bad_vin") == 0) vin = 14.0f;
    if (strcmp(s, "negative_start") == 0) vout = -0.02f;
    buck_control_request = BUCK_CONTROL_CMD_START;
    BuckControl_Process(tick);
    if (strcmp(s, "prebias") == 0 || strcmp(s, "bad_pwm") == 0 || strcmp(s, "bad_vin") == 0
        || strcmp(s, "negative_start") == 0)
    {
        assert(starts == 0U && buck_control_state == BUCK_CONTROL_FAULT);
        printf("PASS start rejection %s\n", s); return 0;
    }
    assert(starts == 1U && running == 1U && buck_control_pwm_active == 1U);
    assert(buck_control_state == BUCK_CONTROL_SOFT_START);
    if (strcmp(s, "overvoltage") == 0) { vout = 11.1f; Sample(); }
    else if (strcmp(s, "adc_loss") == 0) { adc_raw_valid = 0U; Sample(); }
    else if (strcmp(s, "withdraw_ready") == 0) { buck_control_hardware_ready = 0U; Sample(); }
    else if (strcmp(s, "pwm_loss") == 0) { running = 0U; Sample(); }
    else if (strcmp(s, "watchdog") == 0)
    {
        tick += 31U;
        BuckControl_TickGuard(tick);
        assert(running == 0U && buck_control_async_fault != 0U);
        BuckControl_Process(tick);
    }
    else if (strcmp(s, "watchdog_during_update") == 0)
    {
        inject_watchdog = 1U;
        Sample();
        assert(buck_control_async_fault != 0U);
    }
    else if (strcmp(s, "stop") == 0)
    {
        buck_control_request = BUCK_CONTROL_CMD_STOP;
        BuckControl_Process(tick);
        assert(running == 0U && buck_control_state == BUCK_CONTROL_STOPPED);
        puts("PASS manual stop"); return 0;
    }
    else if (strcmp(s, "run") == 0 || strcmp(s, "wrap") == 0)
    {
        vout = 10.0f;
        for (uint32_t i = 0U; i < 220U; ++i) Sample();
        assert(running == 1U && buck_control_state == BUCK_CONTROL_RUNNING);
        puts("PASS manual start and soft-start transition"); return 0;
    }
    else assert(0);
    assert(running == 0U && buck_control_pwm_active == 0U && stops > 0U);
    assert(buck_control_state == BUCK_CONTROL_FAULT && buck_control_fault != 0U);
    buck_control_request = BUCK_CONTROL_CMD_START;
    BuckControl_Process(tick);
    assert(starts == 1U && running == 0U);
    buck_control_request = BUCK_CONTROL_CMD_CLEAR_FAULT;
    BuckControl_Process(tick);
    assert(buck_control_state == BUCK_CONTROL_STOPPED && buck_control_hardware_ready == 0U);
    printf("PASS fault latching/stop/rearm %s\n", s);
#else
    assert(0);
#endif
    return 0;
}
