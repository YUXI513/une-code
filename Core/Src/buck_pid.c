#include "buck_pid.h"
#include <math.h>
#include <string.h>

static float Limit(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static uint32_t ConfigValid(const BuckPIDConfig *c)
{
    if (c == NULL) return 0U;
    return isfinite(c->target_voltage) && c->target_voltage > 0.0f
        && isfinite(c->kp) && c->kp >= 0.0f && isfinite(c->ki) && c->ki >= 0.0f
        && isfinite(c->kd) && c->kd >= 0.0f
        && isfinite(c->duty_min) && c->duty_min > 0.0f
        && isfinite(c->duty_max) && c->duty_max < 1.0f && c->duty_max > c->duty_min
        && isfinite(c->integral_limit) && c->integral_limit > 0.0f
        && isfinite(c->soft_start_volts_per_s) && c->soft_start_volts_per_s > 0.0f
        && isfinite(c->duty_slew_per_s) && c->duty_slew_per_s > 0.0f
        && isfinite(c->derivative_filter_s) && c->derivative_filter_s > 0.0f
        && isfinite(c->vin_min) && c->vin_min > c->target_voltage
        && isfinite(c->vin_max) && c->vin_max > c->vin_min
        && isfinite(c->output_overvoltage) && c->output_overvoltage > c->target_voltage
        && isfinite(c->maximum_step_s) && c->maximum_step_s > 0.0f;
}

BuckPIDConfig BuckPID_DefaultConfig(void)
{
    /* Board values reported by user: L=1 mH, Cout=300 uF, Cin=500 uF;
     * load resistance, ESR, saturation current and current limit unknown.
     * Provisional SLOW bench values, NOT a tuned LC compensator. Nominal
     * 100 Hz execution; retain Kd=0 (PI) until plant/waveforms are measured.
     * Vin margin allows measurement noise at the 15/25 V endpoints. */
    BuckPIDConfig c = {
        .target_voltage = 10.0f, .kp = 0.005f, .ki = 0.05f, .kd = 0.0f,
        .duty_min = 0.08f, .duty_max = 0.80f, .integral_limit = 0.30f,
        .soft_start_volts_per_s = 5.0f, .duty_slew_per_s = 0.50f,
        .derivative_filter_s = 0.02f, .vin_min = 14.5f, .vin_max = 25.5f,
        .output_overvoltage = 11.0f, .maximum_step_s = 0.030f
    };
    return c;
}

uint32_t BuckPID_Reset(BuckPID *pid, const BuckPIDConfig *c)
{
    if (pid == NULL) return 0U;
    memset(pid, 0, sizeof(*pid));
    if (!ConfigValid(c))
    {
        pid->fault = BUCK_PID_FAULT_CONFIG;
        return 0U;
    }
    pid->duty = c->duty_min;
    return 1U;
}

static float Fault(BuckPID *pid, uint32_t fault)
{
    pid->fault |= fault;
    pid->duty = 0.0f;
    pid->integral = 0.0f;
    return 0.0f;
}

static float Step(BuckPID *pid, const BuckPIDConfig *c, float vin,
                   float vout, float dt, uint32_t sample_valid, float minimum_output)
{
    if (pid == NULL) return 0.0f;
    if (pid->fault != 0U) return Fault(pid, 0U);
    if (!ConfigValid(c)) return Fault(pid, BUCK_PID_FAULT_CONFIG);
    if (!isfinite(pid->reference) || !isfinite(pid->integral) || !isfinite(pid->duty)
        || !isfinite(pid->derivative) || !isfinite(pid->previous_output))
        return Fault(pid, BUCK_PID_FAULT_NUMERIC);
    if (!sample_valid || !isfinite(vin) || !isfinite(vout))
        return Fault(pid, BUCK_PID_FAULT_SAMPLE);
    if (!isfinite(dt) || dt <= 0.0f || dt > c->maximum_step_s)
        return Fault(pid, BUCK_PID_FAULT_TIMING);
    if (vin < c->vin_min || vin > c->vin_max)
        return Fault(pid, BUCK_PID_FAULT_INPUT_RANGE);
    if (vout < minimum_output) return Fault(pid, BUCK_PID_FAULT_OUTPUT_RANGE);
    if (vout >= c->output_overvoltage)
        return Fault(pid, BUCK_PID_FAULT_OVERVOLTAGE);

    pid->reference = Limit(pid->reference + c->soft_start_volts_per_s * dt,
                           0.0f, c->target_voltage);
    pid->error = pid->reference - vout;
    float measured_rate = pid->have_previous ? -(vout - pid->previous_output) / dt : 0.0f;
    float alpha = dt / (c->derivative_filter_s + dt);
    pid->derivative += alpha * (measured_rate - pid->derivative);
    pid->previous_output = vout;
    pid->have_previous = 1U;

    /* Input-voltage feedforward + one output-voltage feedback loop.
     * Ki corrects losses/dead-time error; no current loop is implemented. */
    float feedforward = pid->reference / vin;
    float proportional = c->kp * pid->error;
    float derivative = c->kd * pid->derivative;
    float previous_duty = Limit(pid->duty, c->duty_min, c->duty_max);
    float lower = fmaxf(c->duty_min, previous_duty - c->duty_slew_per_s * dt);
    float upper = fminf(c->duty_max, previous_duty + c->duty_slew_per_s * dt);
    pid->integral = Limit(pid->integral, -c->integral_limit, c->integral_limit);
    float integral = Limit(pid->integral + c->ki * pid->error * dt,
                            -c->integral_limit, c->integral_limit);
    float request = feedforward + proportional + integral + derivative;
    /* Conditional integration includes BOTH duty and slew saturation. */
    if (!((request > upper && pid->error > 0.0f)
          || (request < lower && pid->error < 0.0f)))
        pid->integral = integral;
    request = feedforward + proportional + pid->integral + derivative;
    if (!isfinite(request) || !isfinite(pid->integral) || !isfinite(pid->derivative))
        return Fault(pid, BUCK_PID_FAULT_NUMERIC);
    pid->duty = Limit(request, lower, upper);
    return pid->duty;
}

float BuckPID_Step(BuckPID *pid, const BuckPIDConfig *c, float vin,
                   float vout, float dt, uint32_t sample_valid)
{
    return Step(pid, c, vin, vout, dt, sample_valid, 0.0f);
}

float BuckPID_StepStartup(BuckPID *pid, const BuckPIDConfig *c, float vin,
                          float vout, float dt, uint32_t sample_valid)
{
    return Step(pid, c, vin, vout, dt, sample_valid, -BUCK_PID_STARTUP_NEGATIVE_LIMIT_V);
}
