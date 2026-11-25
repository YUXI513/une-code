#include "buck_pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void CheckSteady(float input)
{
    BuckPIDConfig c = BuckPID_DefaultConfig();
    BuckPID pid;
    assert(BuckPID_Reset(&pid, &c));
    float output = 0.0f;
    for (uint32_t step = 0U; step < 2000U; ++step)
    {
        float previous = pid.duty;
        float duty = BuckPID_Step(&pid, &c, input, output, 0.01f, 1U);
        assert(pid.fault == 0U && duty >= c.duty_min && duty <= c.duty_max);
        assert(fabsf(duty - previous) <= c.duty_slew_per_s * 0.01f + 0.00001f);
        /* Simplified DC plant with 30 ms lag and existing dead-time loss.
         * This is a numerical regression, NOT an LC stability guarantee. */
        float plant = input * fmaxf(duty - 320.0f / 8400.0f, 0.0f);
        output += 0.25f * (plant - output);
    }
    assert(fabsf(output - 10.0f) < 0.02f);
    assert(pid.reference == c.target_voltage);
}

int main(void)
{
    CheckSteady(15.0f);
    CheckSteady(20.0f);
    CheckSteady(25.0f);
    BuckPIDConfig c = BuckPID_DefaultConfig();
    BuckPID pid;
    assert(BuckPID_Reset(&pid, &c));
    float y = 0.0f;
    for (uint32_t step = 0U; step < 1800U; ++step)
    {
        float vin = 15.0f + fminf((float)step / 100.0f, 10.0f);
        float duty = BuckPID_Step(&pid, &c, vin, y, 0.01f, 1U);
        assert(pid.fault == 0U);
        y += 0.25f * (vin * fmaxf(duty - 320.0f / 8400.0f, 0.0f) - y);
        if (step > 500U) assert(fabsf(y - 10.0f) < 0.15f);
    }
    assert(fabsf(y - 10.0f) < 0.02f);
    /* Saturation does not accumulate an unbounded integral. */
    assert(BuckPID_Reset(&pid, &c));
    for (uint32_t step = 0U; step < 3000U; ++step)
    {
        (void)BuckPID_Step(&pid, &c, 15.0f, 0.0f, 0.01f, 1U);
        assert(pid.fault == 0U && fabsf(pid.integral) <= c.integral_limit);
    }
    assert(pid.duty <= 0.8f);
    /* Proper feedback sign after soft start, without changing the target. */
    c.ki = 0.0f;
    assert(BuckPID_Reset(&pid, &c));
    pid.reference = 10.0f; pid.duty = 0.5f;
    assert(BuckPID_Step(&pid, &c, 20.0f, 9.0f, 0.01f, 1U) > 0.5f);
    assert(BuckPID_Reset(&pid, &c));
    pid.reference = 10.0f; pid.duty = 0.5f;
    assert(BuckPID_Step(&pid, &c, 20.0f, 10.5f, 0.01f, 1U) < 0.5f);

    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, 11.0f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_OVERVOLTAGE);
    assert(BuckPID_Step(&pid, &c, 20.0f, 9.0f, 0.01f, 1U) == 0.0f);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 26.0f, 0.0f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_INPUT_RANGE);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, -0.1f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_OUTPUT_RANGE);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, NAN, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_SAMPLE);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, 0.0f, 0.031f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_TIMING);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, 0.0f, 0.01f, 0U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_SAMPLE);
    assert(BuckPID_Reset(&pid, &c));
    pid.duty = NAN;
    assert(BuckPID_Step(&pid, &c, 20.0f, 0.0f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_NUMERIC);
    assert(BuckPID_Reset(&pid, &c));
    pid.duty = 0.8f;
    c.duty_max = 0.5f;
    assert(BuckPID_Step(&pid, &c, 20.0f, 9.0f, 0.01f, 1U) <= 0.5f);
    c = BuckPID_DefaultConfig();
    c.kd = 0.001f;
    assert(BuckPID_Reset(&pid, &c));
    (void)BuckPID_Step(&pid, &c, 20.0f, 0.1f, 0.01f, 1U);
    assert(pid.derivative == 0.0f);
    (void)BuckPID_Step(&pid, &c, 20.0f, 0.1f, 0.01f, 1U);
    assert(pid.derivative == 0.0f); /* Setpoint ramp has no derivative kick. */
    c.kp = NAN;
    assert(!BuckPID_Reset(&pid, &c));
    c = BuckPID_DefaultConfig();
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_StepStartup(&pid, &c, 20.0f, -0.45f, 0.01f, 1U) > 0.0f);
    assert(pid.fault == 0U && fabsf(pid.error - 0.50f) < 0.0001f);
    /* It used the signed -0.45 V, not an artificial zero measurement. */
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_StepStartup(&pid, &c, 20.0f, -0.501f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_OUTPUT_RANGE);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_Step(&pid, &c, 20.0f, -0.01f, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_OUTPUT_RANGE);
    assert(BuckPID_Reset(&pid, &c));
    assert(BuckPID_StepStartup(&pid, &c, 20.0f, NAN, 0.01f, 1U) == 0.0f);
    assert(pid.fault & BUCK_PID_FAULT_SAMPLE);
    puts("PASS PID: 15/20/25 V DC models, slow Vin ramp, soft start, slew, feedback sign, anti-windup and latched faults");
    return 0;
}
