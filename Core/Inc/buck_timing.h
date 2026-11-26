#ifndef BUCK_TIMING_H
#define BUCK_TIMING_H

/* Keep the PWM quiet-window calculation aligned with the 11-rank ADC scan.
 * 12.5-cycle acquisition + 12.5-cycle 12-bit conversion per rank. */
#define BUCK_ADC_REGULAR_RANK_COUNT   11U
#define BUCK_ADC_SAMPLE_CYCLES_X2     25U
#define BUCK_ADC_CONVERSION_CYCLES_X2 25U
#define BUCK_ADC_SEQUENCE_CYCLES_X2 \
    ((BUCK_ADC_SAMPLE_CYCLES_X2 + BUCK_ADC_CONVERSION_CYCLES_X2) \
     * BUCK_ADC_REGULAR_RANK_COUNT)

#endif
