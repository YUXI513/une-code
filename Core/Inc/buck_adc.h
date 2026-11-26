#ifndef BUCK_ADC_H
#define BUCK_ADC_H

#include "stm32g4xx_hal.h"

/* No jumper: bench-test the INTERNAL 2.5 V reference; default PWM is OFF.
 * The actual Cref+ capacitor is unknown (schematic: 2.2 uF, outside the
 * datasheet's 0.5..1.5 uF load range, which also requires 100 nF low ESR).
 * A ready flag or a DMM reading does NOT validate stability/ADC accuracy.
 * User-authorized bench readings use the nominal 2.5 V and are estimates.
 * BOTH confirmations below remain OFF until the hardware/reference are
 * verified. Normal control getters remain NAN. Estimate getters provide
 * fresh nominal readings, marked '~' on the OLED. Only the separate,
 * explicit BUCK_CONTROL_NOMINAL_BENCH targets may use these estimates
 * for limited bench control (manual, or user-requested qualified AUTO).
 * Neither mode certifies the reference/hardware.
 * Do not externally power VREF+ while running this internal-reference test.
 * VDDA was measured as 3.3 V. Internal 2.5 V requires VDDA >= 2.8 V.
 * PWM uses PA10/PB15. Retired PB13 stays LOW to avoid the ES0430 2.8.1
 * VREFBUF disturbance; moving pins does not validate the reference capacitor.
 * Setting the test macro to 0 retains external-reference initialization. */
#ifndef BUCK_ADC_INTERNAL_REFERENCE_TEST
#define BUCK_ADC_INTERNAL_REFERENCE_TEST 1U
#endif
#if BUCK_ADC_INTERNAL_REFERENCE_TEST && defined(ADC_ONLY_TEST) && !ADC_ONLY_TEST \
    && (!defined(BUCK_CONTROL_NOMINAL_BENCH) || !BUCK_CONTROL_NOMINAL_BENCH)
#error "Internal reference diagnostic requires ADC_ONLY_TEST=ON; PWM must stay off"
#endif
#define BUCK_ADC_REFERENCE_CONFIRMED  0U
#define BUCK_ADC_REFERENCE_HARDWARE_CONFIRMED 0U
#if BUCK_ADC_INTERNAL_REFERENCE_TEST
#define BUCK_ADC_REFERENCE_VOLTS      2.5f
#else
#define BUCK_ADC_REFERENCE_VOLTS      3.3f
#endif
#define BUCK_ADC_VDDA_VOLTS           3.3f
#define BUCK_ADC_INPUT_DIVIDER_RATIO  40.0f
#define BUCK_ADC_OUTPUT_DIVIDER_RATIO 40.0f
#define BUCK_ADC_UPDATE_MS            10U
#define BUCK_ADC_AVERAGE_COUNT        16U

/* Bitmask: reference unknown is expected until the hardware is confirmed. */
#define BUCK_ADC_ERR_REFERENCE        (1UL << 0)
#define BUCK_ADC_ERR_INIT             (1UL << 1)
#define BUCK_ADC_ERR_CLOCK            (1UL << 2)
#define BUCK_ADC_ERR_CALIBRATION      (1UL << 3)
#define BUCK_ADC_ERR_CHANNEL          (1UL << 4)
#define BUCK_ADC_ERR_START            (1UL << 5)
#define BUCK_ADC_ERR_TIMEOUT          (1UL << 6)
#define BUCK_ADC_ERR_SEQUENCE         (1UL << 7)
#define BUCK_ADC_ERR_OVERRUN          (1UL << 8)
#define BUCK_ADC_ERR_STOP             (1UL << 9)
#define BUCK_ADC_ERR_RAIL             (1UL << 10)
#define BUCK_ADC_ERR_NEGATIVE         (1UL << 11)
#define BUCK_ADC_ERR_STALE            (1UL << 12)
#define BUCK_ADC_ERR_CONVERSION       (1UL << 13)
#define BUCK_ADC_ERR_REFERENCE_MODE   (1UL << 14)
#define BUCK_ADC_ERR_VREFBUF_READY    (1UL << 15)

/* Two identical AMC1311 voltage boards, each with its own U+/U- output.
 * VIN: PA0/ADC1_IN1 (+), PA1/ADC1_IN2 (-), header pins 3/4.
 * VOUT: PA2/ADC1_IN3 (+), PA3/ADC1_IN4 (-), header pins 5/6.
 * Both theoretical divider ratios are 40; calibrate them independently.
 * Do not connect either board's I+/I- (current sensing) to these inputs.
 * Legacy adc_raw/voltage_plus/minus names now refer to VOUT on PA2/PA3.
 * Raw Watch values are rounded averages. Voltage calculations retain the
 * fractional average; buck_input/output_voltage are NOT display-smoothed. */
extern volatile uint32_t adc_raw_plus, adc_raw_minus;
extern volatile uint32_t adc_input_raw_plus, adc_input_raw_minus;
extern volatile float adc_voltage_plus, adc_voltage_minus, adc_voltage_diff;
extern volatile float adc_input_voltage_plus, adc_input_voltage_minus, adc_input_voltage_diff;
extern volatile float buck_input_voltage, buck_output_voltage;
extern volatile uint32_t adc_raw_valid, adc_sample_valid, adc_error_code;
extern volatile uint32_t adc_input_sample_valid, adc_output_sample_valid;
extern volatile uint32_t adc_input_estimate_valid, adc_output_estimate_valid;
extern volatile uint32_t adc_input_error_code, adc_output_error_code;
extern volatile uint32_t adc_hal_error, adc_last_hal_status, adc_failure_count;
extern volatile uint32_t adc_update_count, adc_last_sample_ms;
extern volatile uint32_t adc_clock_hz, adc_vrefbuf_csr;
extern volatile uint32_t adc_vrefbuf_ready, adc_vrefbuf_startup_status;
extern volatile uint32_t adc_vrefbuf_fault_csr;
extern volatile float adc_reference_voltage, adc_vdda_voltage;
extern volatile uint32_t adc_reference_confirmed, adc_reference_hardware_confirmed;

typedef struct {
    uint32_t minimum, maximum, zero_count, full_count, samples;
} BuckADCRawStats;

typedef struct {
    uint32_t valid, timestamp_ms, batch_number, frame_number;
    uint32_t raw[4]; /* First input-rail frame: PA0, PA1, PA2, PA3. */
    uint32_t endpoint_mask; /* A0 zero/full, A1 zero/full in bits 0..3. */
    uint32_t input_plus_confirm;
    uint32_t rank_raw[11]; /* All conversion ranks from that same frame. */
    BuckADCRawStats input_stats[2]; /* Triggering batch ranges/counts. */
    uint32_t hal_status, error_code;
} BuckADCFirstInputRail;

/* Diagnostic only: retained input endpoint totals and first rail frame. */
extern volatile BuckADCRawStats adc_input_stats[2];
extern volatile uint32_t adc_input_full_total[2], adc_input_zero_total[2];
extern volatile uint32_t adc_diagnostic_batches;
extern volatile BuckADCFirstInputRail adc_first_input_rail;

/* HAL_OK from ReadVoltage means all four fresh raw readings were acquired.
 * adc_sample_valid requires BOTH voltage pairs valid. Each voltage getter
 * checks its own pair validity and freshness; unknown reference,
 * saturation and negative differential results are explicitly invalid.
 * Call from main context only, with SysTick running. No IRQ/DMA is used. */
HAL_StatusTypeDef BuckADC_Init(void);
HAL_StatusTypeDef BuckADC_ReadVoltage(void);
uint32_t BuckADC_GetRawPlus(void);
uint32_t BuckADC_GetRawMinus(void);
float BuckADC_GetDifferentialVoltage(void);
float BuckADC_GetOutputVoltage(void);
float BuckADC_GetInputVoltage(void);
/* Nominal estimates, not verified feedback. Normal control must reject them.
 * The explicit BUCK_CONTROL_NOMINAL_BENCH adapter is the sole bench-control
 * exception. They must never be treated as evidence that hardware is ready. */
float BuckADC_GetInputVoltageEstimate(void);
float BuckADC_GetOutputVoltageEstimate(void);
/* Signed bench-start estimate, not normally valid feedback. Retains negative
 * values/flags; caller MUST enforce amplitude and startup-time bounds. */
float BuckADC_GetOutputVoltageStartupEstimate(void);

/* OLED diagnostics only: retain fresh signed readings on NEGATIVE DIFF.
 * Unknown reference is permitted only in the existing nominal bench mode.
 * Conversion, rail, reference-mode and stale faults still return NAN.
 * NEVER use these getters for feedback or to decide that PWM may start. */
float BuckADC_GetInputVoltageForDisplay(void);
float BuckADC_GetOutputVoltageForDisplay(void);

#endif
