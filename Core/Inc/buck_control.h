#ifndef BUCK_CONTROL_H
#define BUCK_CONTROL_H

#include "buck_pid.h"

#ifndef BUCK_CONTROL_POWER_STAGE
#define BUCK_CONTROL_POWER_STAGE 0
#endif
#ifndef BUCK_CONTROL_NOMINAL_BENCH
#define BUCK_CONTROL_NOMINAL_BENCH 0
#endif
#ifndef BUCK_CONTROL_AUTO_START
#define BUCK_CONTROL_AUTO_START 0
#endif
#if BUCK_CONTROL_AUTO_START && !BUCK_CONTROL_POWER_STAGE
#error "Automatic startup requires a dedicated power-stage controller build"
#endif

#define BUCK_CONTROL_AUTO_READY_MS 1000U
#define BUCK_CONTROL_AUTO_READY_SAMPLES 100U
#define BUCK_CONTROL_STARTUP_NEGATIVE_MS 200U

enum
{
    BUCK_CONTROL_PREVIEW = 0,
    BUCK_CONTROL_STOPPED,
    BUCK_CONTROL_SOFT_START,
    BUCK_CONTROL_RUNNING,
    BUCK_CONTROL_INHIBITED,
    BUCK_CONTROL_FAULT,
    BUCK_CONTROL_WAIT_START
};

enum
{
    BUCK_CONTROL_WAIT_ADC = 1,
    BUCK_CONTROL_WAIT_VIN,
    BUCK_CONTROL_WAIT_VOUT,
    BUCK_CONTROL_WAIT_QUALIFY
};
enum
{
    BUCK_CONTROL_CMD_NONE = 0,
    BUCK_CONTROL_CMD_PREVIEW = 1,
    BUCK_CONTROL_CMD_START = 2,
    BUCK_CONTROL_CMD_STOP = 3,
    BUCK_CONTROL_CMD_CLEAR_FAULT = 4
};
enum
{
    BUCK_CONTROL_ERR_ADC = 1U << 8,
    BUCK_CONTROL_ERR_PWM = 1U << 9,
    BUCK_CONTROL_ERR_DEADLINE = 1U << 10,
    BUCK_CONTROL_ERR_INTERLOCK = 1U << 11,
    BUCK_CONTROL_ERR_PREBIAS = 1U << 12,
    BUCK_CONTROL_ERR_BUILD_MODE = 1U << 13,
    BUCK_CONTROL_ERR_STARTUP_VOUT = 1U << 14
};

/* Why feedback was rejected; separate from low-level ADC conversion flags.
 * A healthy raw frame can still be too old for the controller. */
enum
{
    BUCK_CONTROL_FEEDBACK_NO_RAW = 1U << 0,
    BUCK_CONTROL_FEEDBACK_STALE = 1U << 1,
    BUCK_CONTROL_FEEDBACK_VIN = 1U << 2,
    BUCK_CONTROL_FEEDBACK_VOUT = 1U << 3
};

extern BuckPIDConfig buck_control_config;
extern volatile uint32_t buck_control_state, buck_control_fault;
extern volatile uint32_t buck_control_async_fault;
extern volatile uint32_t buck_control_request, buck_control_pwm_active;
/* Default DEMO is calculation-only and rejects START.
 * DEMO_BUCK_CONTROL_TEST is separate, manually armed bench firmware:
 *  - no automatic start after reset or after a fault;
 *  - only after real hardware checks: hardware_ready=1, request=CMD_START;
 *  - request=CMD_STOP stops; CMD_CLEAR_FAULT also clears hardware_ready.
 * Edit requests only with PWM off; do not halt/debug a powered controller.
 * Keep an independent power disconnect/current limit available. */
/* Stays 0: operator must verify the actual gate dead times across the duty
 * range, correct feedback, a known load and a current-limited supply before
 * setting 1 and requesting START. Reset always disarms it again.
 * A single voltage loop is NOT overcurrent protection. */
extern volatile uint32_t buck_control_hardware_ready;
/* User-requested AUTO build grants software startup permission after fresh,
 * valid feedback qualifies for 1 s; it does NOT set hardware_ready or certify
 * real gate timing, load ratings, reference accuracy or current protection.
 * Manual/default targets retain their original behavior. AUTO never retries
 * after a running fault, STOP, PREVIEW or CLEAR_FAULT until reset. */
extern volatile uint32_t buck_control_auto_pending;
extern volatile uint32_t buck_control_auto_samples, buck_control_auto_wait;
/* AUTO-only bench startup can use fresh signed VOUT >= -0.50 V, for at
 * most 200 ms after PWM starts; closes early above +0.5 V. No calibration
 * or clamping occurs. All ADC flags/raw values remain visible/unchanged. */
extern volatile uint32_t buck_control_startup_negative;

typedef struct
{
    uint32_t valid, control_error, source_state, feedback_error;
    uint32_t input_error, output_error, raw_valid;
    uint32_t input_raw_plus, input_raw_minus, output_raw_plus, output_raw_minus;
    uint32_t input_full_total[2];
    uint32_t input_zero_total[2];
    uint32_t elapsed_ms, sample_age_ms, start_attempts, negative_permitted;
    float input_voltage, output_voltage;
} BuckControlFaultSnapshot;
/* First fault only; never overwritten by later successful ADC samples or
 * repeated watchdog handling. Reset/CLEAR_FAULT clears valid. Signed,
 * unsmoothed voltage and ADC flags are frozen together for OLED/Watch. */
extern volatile BuckControlFaultSnapshot buck_control_fault_snapshot;
extern volatile uint32_t buck_control_start_attempts;
extern volatile uint32_t buck_control_feedback_estimate;
extern volatile uint32_t buck_control_last_step_ms, buck_control_steps;
extern volatile float buck_control_target, buck_control_reference, buck_control_error;
extern volatile float buck_control_duty, buck_control_integral;
extern volatile float buck_control_vin, buck_control_vout;

void BuckControl_Init(void);
/* Call immediately after each ADC attempt, once per fresh 10 ms sample. */
void BuckControl_Process(uint32_t now);
/* Fast SysTick guard: stops on a stalled loop when interrupts still run.
 * It cannot protect while the CPU/debugger is halted or interrupts blocked. */
void BuckControl_TickGuard(uint32_t now);
const char *BuckControl_StateText(void);

#endif
