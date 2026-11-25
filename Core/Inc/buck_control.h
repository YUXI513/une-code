#ifndef BUCK_CONTROL_H
#define BUCK_CONTROL_H

#include "buck_pid.h"

#ifndef BUCK_CONTROL_POWER_STAGE
#define BUCK_CONTROL_POWER_STAGE 0
#endif
#ifndef BUCK_CONTROL_NOMINAL_BENCH
#define BUCK_CONTROL_NOMINAL_BENCH 0
#endif

enum
{
    BUCK_CONTROL_PREVIEW = 0,
    BUCK_CONTROL_STOPPED,
    BUCK_CONTROL_SOFT_START,
    BUCK_CONTROL_RUNNING,
    BUCK_CONTROL_INHIBITED,
    BUCK_CONTROL_FAULT
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
    BUCK_CONTROL_ERR_BUILD_MODE = 1U << 13
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
 * A single voltage loop and its slow OVP are NOT overcurrent protection. */
extern volatile uint32_t buck_control_hardware_ready;
extern volatile uint32_t buck_control_feedback_estimate;
extern volatile uint32_t buck_control_last_step_ms, buck_control_steps;
extern volatile float buck_control_reference, buck_control_error;
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
