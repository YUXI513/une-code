#include "buck_control.h"
#include "buck_adc.h"
#include "buck_pwm.h"
#include <math.h>

#if BUCK_CONTROL_POWER_STAGE && ADC_ONLY_TEST
#error "Power-stage control requires the explicit control test target"
#endif
#if BUCK_CONTROL_POWER_STAGE && defined(BUCK_PWM_TEST_PA10_ONLY)
#error "A PA10-only diagnostic build must never run the Buck controller"
#endif

#define CONTROL_STALE_MS 30U
#define CONTROL_START_MAX_VOUT 0.5f

BuckPIDConfig buck_control_config;
volatile uint32_t buck_control_state, buck_control_fault;
volatile uint32_t buck_control_async_fault;
volatile uint32_t buck_control_request, buck_control_pwm_active;
volatile uint32_t buck_control_hardware_ready;
volatile uint32_t buck_control_auto_pending;
volatile uint32_t buck_control_auto_samples, buck_control_auto_wait;
volatile uint32_t buck_control_feedback_estimate;
volatile uint32_t buck_control_last_step_ms, buck_control_steps;
volatile float buck_control_reference, buck_control_error;
volatile float buck_control_duty, buck_control_integral;
volatile float buck_control_vin = NAN, buck_control_vout = NAN;

static BuckPID controller;
static uint32_t last_sequence;
static uint32_t need_reset;
static uint32_t automatic_permit;
#if BUCK_CONTROL_AUTO_START
static uint32_t auto_last_sequence, auto_first_good_ms, auto_last_good_ms;
#endif

static uint32_t StartPermitted(void)
{
    return buck_control_hardware_ready == 1U || automatic_permit != 0U;
}

static void StopOutputs(void)
{
#if BUCK_CONTROL_POWER_STAGE
    BuckPWM_Stop();
#endif
    buck_control_pwm_active = 0U;
}

static void SetFault(uint32_t reason)
{
    StopOutputs();
    buck_control_auto_pending = 0U;
    automatic_permit = 0U;
    buck_control_duty = 0.0f;
    buck_control_fault |= reason;
    buck_control_state = BUCK_CONTROL_FAULT;
}

static void Snapshot(void)
{
    buck_control_reference = controller.reference;
    buck_control_error = controller.error;
    buck_control_integral = controller.integral;
    buck_control_duty = controller.duty;
}

static uint32_t ReadFeedback(uint32_t now, uint32_t allow_estimate)
{
    buck_control_feedback_estimate = 0U;
    if (adc_raw_valid == 0U || (uint32_t)(now - adc_last_sample_ms) > CONTROL_STALE_MS)
    {
        buck_control_vin = NAN;
        buck_control_vout = NAN;
        return 0U;
    }
    float vin = BuckADC_GetInputVoltage();
    float vout = BuckADC_GetOutputVoltage();
    if ((!isfinite(vin) || !isfinite(vout)) && allow_estimate)
    {
        vin = BuckADC_GetInputVoltageEstimate();
        vout = BuckADC_GetOutputVoltageEstimate();
        buck_control_feedback_estimate = 1U;
    }
    buck_control_vin = vin;
    buck_control_vout = vout;
    return isfinite(vin) && isfinite(vout);
}

void BuckControl_Init(void)
{
    StopOutputs();
    buck_control_config = BuckPID_DefaultConfig();
    (void)BuckPID_Reset(&controller, &buck_control_config);
    buck_control_state = BUCK_CONTROL_PREVIEW;
    buck_control_fault = 0U;
    buck_control_async_fault = 0U;
    buck_control_hardware_ready = 0U;
    automatic_permit = 0U;
    buck_control_auto_pending = 0U;
    buck_control_auto_samples = 0U;
    buck_control_auto_wait = BUCK_CONTROL_WAIT_ADC;
#if BUCK_CONTROL_AUTO_START
    buck_control_state = BUCK_CONTROL_WAIT_START;
    buck_control_auto_pending = 1U;
    auto_last_sequence = adc_update_count;
    auto_first_good_ms = 0U;
    auto_last_good_ms = 0U;
#endif
    buck_control_request = BUCK_CONTROL_CMD_NONE;
    buck_control_last_step_ms = HAL_GetTick();
    buck_control_steps = 0U;
    last_sequence = adc_update_count;
    need_reset = 1U;
    Snapshot();
    buck_control_duty = 0.0f;
}

static void HandleCommand(uint32_t now)
{
    uint32_t command = buck_control_request;
    buck_control_request = BUCK_CONTROL_CMD_NONE;
    if (command == BUCK_CONTROL_CMD_NONE) return;
    if (command == BUCK_CONTROL_CMD_STOP || command == BUCK_CONTROL_CMD_PREVIEW)
    {
        StopOutputs();
        buck_control_auto_pending = 0U;
        automatic_permit = 0U;
        buck_control_duty = 0.0f;
        need_reset = 1U;
        /* STOP never clears a latched fault. */
        if (buck_control_state != BUCK_CONTROL_FAULT)
            buck_control_state = command == BUCK_CONTROL_CMD_STOP
                ? BUCK_CONTROL_STOPPED : BUCK_CONTROL_PREVIEW;
        return;
    }
    if (command == BUCK_CONTROL_CMD_CLEAR_FAULT)
    {
        StopOutputs();
        buck_control_auto_pending = 0U;
        automatic_permit = 0U;
        buck_control_hardware_ready = 0U;
        buck_control_fault = 0U;
        buck_control_async_fault = 0U;
        buck_control_state = BUCK_CONTROL_STOPPED;
        buck_control_duty = 0.0f;
        need_reset = 1U;
        return;
    }
    if (command != BUCK_CONTROL_CMD_START || buck_control_state == BUCK_CONTROL_FAULT)
        return;
    buck_control_auto_pending = 0U;
    buck_control_duty = 0.0f;
#if !BUCK_CONTROL_POWER_STAGE
    (void)now;
    buck_control_fault = BUCK_CONTROL_ERR_BUILD_MODE;
    buck_control_state = BUCK_CONTROL_INHIBITED;
#else
    StopOutputs();
    if (!StartPermitted())
    {
        buck_control_fault = BUCK_CONTROL_ERR_INTERLOCK;
        buck_control_state = BUCK_CONTROL_INHIBITED;
        return;
    }
    if (!ReadFeedback(now, BUCK_CONTROL_NOMINAL_BENCH))
    {
        SetFault(BUCK_CONTROL_ERR_ADC);
        return;
    }
    /* Avoid immediately driving the low MOSFET into a charged output. */
    if (buck_control_vout > CONTROL_START_MAX_VOUT || buck_control_vout < 0.0f)
    {
        SetFault(BUCK_CONTROL_ERR_PREBIAS);
        return;
    }
    if (!BuckPID_Reset(&controller, &buck_control_config))
    {
        SetFault(controller.fault);
        return;
    }
    float duty = BuckPID_Step(&controller, &buck_control_config,
                              buck_control_vin, buck_control_vout,
                              (float)BUCK_ADC_UPDATE_MS * 0.001f, 1U);
    if (controller.fault != 0U)
    {
        SetFault(controller.fault);
        return;
    }
    if (!BuckPWM_SetDuty(duty))
    {
        SetFault(BUCK_CONTROL_ERR_PWM);
        return;
    }
    buck_control_fault = 0U;
    buck_control_last_step_ms = HAL_GetTick();
    buck_control_pwm_active = 1U;
    buck_control_state = BUCK_CONTROL_SOFT_START;
    BuckPWM_Start();
    if (!BuckPWM_IsRunning() || buck_control_async_fault != 0U)
    {
        SetFault(BUCK_CONTROL_ERR_PWM);
        return;
    }
    need_reset = 0U;
    last_sequence = adc_update_count;
    Snapshot();
#endif
}

#if BUCK_CONTROL_AUTO_START
static void ResetAutoQualification(uint32_t reason)
{
    buck_control_auto_samples = 0U;
    buck_control_auto_wait = reason;
}

static void ProcessAutomaticStart(uint32_t now)
{
    /* Keep PWM off while waiting; neither old frames nor repeated calls
     * count toward qualification. Negative differential is still invalid. */
    if (!buck_control_auto_pending) return;
    if (!ReadFeedback(now, BUCK_CONTROL_NOMINAL_BENCH))
    {
        ResetAutoQualification(BUCK_CONTROL_WAIT_ADC);
        return;
    }
    if (buck_control_vin < 15.0f || buck_control_vin > 25.0f)
    {
        ResetAutoQualification(BUCK_CONTROL_WAIT_VIN);
        return;
    }
    if (buck_control_vout < 0.0f || buck_control_vout > CONTROL_START_MAX_VOUT)
    {
        ResetAutoQualification(BUCK_CONTROL_WAIT_VOUT);
        return;
    }
    if (adc_update_count == auto_last_sequence) return;
    auto_last_sequence = adc_update_count;
    if (buck_control_auto_samples == 0U
        || (uint32_t)(now - auto_last_good_ms) > CONTROL_STALE_MS)
    {
        buck_control_auto_samples = 0U;
        auto_first_good_ms = now;
    }
    auto_last_good_ms = now;
    buck_control_auto_wait = BUCK_CONTROL_WAIT_QUALIFY;
    if (buck_control_auto_samples < BUCK_CONTROL_AUTO_READY_SAMPLES)
        ++buck_control_auto_samples;
    if (buck_control_auto_samples < BUCK_CONTROL_AUTO_READY_SAMPLES
        || (uint32_t)(now - auto_first_good_ms) < BUCK_CONTROL_AUTO_READY_MS) return;

    /* Explicit AUTO target, authorized by user. Reuse all normal START
     * checks; do not pretend the operator's hardware_ready flag is set. */
    buck_control_auto_pending = 0U;
    automatic_permit = 1U;
    buck_control_request = BUCK_CONTROL_CMD_START;
    HandleCommand(now);
}
#endif

void BuckControl_Process(uint32_t now)
{
    /* Only main writes the state machine. SysTick has its own stop latch so
     * an interrupted RUNNING assignment cannot overwrite a watchdog fault. */
    if (buck_control_async_fault != 0U) SetFault(buck_control_async_fault);
    HandleCommand(now);
#if BUCK_CONTROL_AUTO_START
    if (buck_control_state == BUCK_CONTROL_WAIT_START)
    {
        ProcessAutomaticStart(now);
        return;
    }
#endif
    uint32_t active = buck_control_pwm_active;
    if (active && !StartPermitted())
    {
        SetFault(BUCK_CONTROL_ERR_INTERLOCK);
        return;
    }
    if (buck_control_state == BUCK_CONTROL_FAULT || buck_control_state == BUCK_CONTROL_STOPPED
        || buck_control_state == BUCK_CONTROL_INHIBITED) return;
    uint32_t preview = buck_control_state == BUCK_CONTROL_PREVIEW;
    if (!ReadFeedback(now, preview || BUCK_CONTROL_NOMINAL_BENCH))
    {
        if (active) SetFault(BUCK_CONTROL_ERR_ADC);
        else
        {
            need_reset = 1U;
            buck_control_duty = 0.0f;
            buck_control_fault = BUCK_CONTROL_ERR_ADC;
        }
        return;
    }
    if (adc_update_count == last_sequence) return;
    last_sequence = adc_update_count;
    float dt = (float)(uint32_t)(now - buck_control_last_step_ms) * 0.001f;
    if (need_reset)
    {
        (void)BuckPID_Reset(&controller, &buck_control_config);
        dt = (float)BUCK_ADC_UPDATE_MS * 0.001f;
        need_reset = 0U;
    }
    float duty = BuckPID_Step(&controller, &buck_control_config,
                              buck_control_vin, buck_control_vout, dt, 1U);
    buck_control_last_step_ms = now;
    ++buck_control_steps;
    if (controller.fault != 0U)
    {
        if (active) SetFault(controller.fault);
        else { buck_control_fault = controller.fault; need_reset = 1U; buck_control_duty = 0.0f; }
        return;
    }
#if BUCK_CONTROL_POWER_STAGE
    if (active)
    {
        if (buck_control_async_fault != 0U || !BuckPWM_IsRunning() || !BuckPWM_SetDuty(duty))
        {
            SetFault(BUCK_CONTROL_ERR_PWM);
            return;
        }
        if (buck_control_async_fault != 0U)
        {
            SetFault(buck_control_async_fault);
            return;
        }
        buck_control_state = controller.reference < buck_control_config.target_voltage
            ? BUCK_CONTROL_SOFT_START : BUCK_CONTROL_RUNNING;
    }
#else
    (void)duty;
#endif
    if (!active) buck_control_fault = 0U;
    if (buck_control_async_fault != 0U)
    {
        SetFault(buck_control_async_fault);
        return;
    }
    Snapshot();
}

void BuckControl_TickGuard(uint32_t now)
{
#if BUCK_CONTROL_POWER_STAGE
    if (buck_control_pwm_active != 0U
        && (uint32_t)(now - buck_control_last_step_ms) > CONTROL_STALE_MS)
    {
        buck_control_async_fault |= BUCK_CONTROL_ERR_DEADLINE;
        BuckPWM_Stop();
        buck_control_pwm_active = 0U;
    }
#else
    (void)now;
#endif
}

const char *BuckControl_StateText(void)
{
    if (buck_control_async_fault != 0U) return "PID FAULT PWM OFF";
    switch (buck_control_state)
    {
        case BUCK_CONTROL_WAIT_START:
            switch (buck_control_auto_wait)
            {
                case BUCK_CONTROL_WAIT_VIN: return "AUTO WAIT VIN 15-25V";
                case BUCK_CONTROL_WAIT_VOUT: return "AUTO WAIT VOUT 0-0.5";
                case BUCK_CONTROL_WAIT_QUALIFY: return "AUTO CHECK 1S";
                default: return "AUTO WAIT ADC";
            }
        case BUCK_CONTROL_PREVIEW: return buck_control_fault != 0U
            ? "PID PREVIEW INVALID" : "PID PREVIEW PWM OFF";
        case BUCK_CONTROL_STOPPED: return "PID STOPPED PWM OFF";
        case BUCK_CONTROL_SOFT_START: return "PID SOFT START";
        case BUCK_CONTROL_RUNNING: return buck_control_feedback_estimate
            ? "PID RUN NOMINAL VREF" : "PID RUNNING";
        case BUCK_CONTROL_INHIBITED: return "PID START INHIBITED";
        default: return "PID FAULT PWM OFF";
    }
}
