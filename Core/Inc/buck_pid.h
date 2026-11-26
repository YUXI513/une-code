#ifndef BUCK_PID_H
#define BUCK_PID_H

#include <stdint.h>

/* Duty is a fraction, not percent. Kp: duty/V; Ki: duty/(V*s);
 * Kd: duty*s/V. Derivative is on measurement, with a first-order filter.
 * Pure numerical controller: no peripheral access and no automatic start. */
typedef struct
{
    float output_ratio; /* Desired VOUT/VIN, default 0.5. */
    float kp, ki, kd;
    float duty_min, duty_max;
    float integral_limit;
    float soft_start_volts_per_s;
    float duty_slew_per_s;
    float derivative_filter_s;
    float output_overvoltage; /* Absolute output limit, independent of VIN. */
    float maximum_step_s;
} BuckPIDConfig;

typedef struct
{
    float target_voltage; /* Current VIN * output_ratio, before soft start. */
    float reference, error, integral, derivative, duty;
    float previous_output;
    uint32_t have_previous;
    uint32_t soft_start_complete;
    uint32_t fault;
} BuckPID;

enum
{
    BUCK_PID_FAULT_CONFIG = 1U << 0,
    BUCK_PID_FAULT_SAMPLE = 1U << 1,
    BUCK_PID_FAULT_INPUT_NONPOSITIVE = 1U << 2,
    BUCK_PID_FAULT_OUTPUT_RANGE = 1U << 3,
    BUCK_PID_FAULT_OVERVOLTAGE = 1U << 4,
    BUCK_PID_FAULT_TIMING = 1U << 5,
    BUCK_PID_FAULT_NUMERIC = 1U << 6
};

BuckPIDConfig BuckPID_DefaultConfig(void);
uint32_t BuckPID_Reset(BuckPID *pid, const BuckPIDConfig *config);
/* A fault latches until Reset. Fault return is 0 duty, meaning ALL OFF;
 * the hardware adapter must stop PWM, never encode it as a 0% CHx pair. */
float BuckPID_Step(BuckPID *pid, const BuckPIDConfig *config, float vin,
                   float vout, float elapsed_s, uint32_t sample_valid);

/* Explicit bench-start exception; no zero clamp or offset compensation.
 * The adapter must enforce its 200 ms window and normal strict checks after
 * output rises. Ordinary BuckPID_Step still rejects EVERY negative voltage. */
#define BUCK_PID_STARTUP_NEGATIVE_LIMIT_V 0.50f
float BuckPID_StepStartup(BuckPID *pid, const BuckPIDConfig *config, float vin,
                          float vout, float elapsed_s, uint32_t sample_valid);

#endif
