#ifndef BUCK_TIMING_H
#define BUCK_TIMING_H

/* Keep the ADC sequence and PWM quiet-window calculation in agreement.
 * 24.5-cycle acquisition plus 12.5-cycle 12-bit conversion, for nine ranks. */
#define BUCK_ADC_REGULAR_RANK_COUNT   9U
#define BUCK_ADC_SAMPLE_CYCLES_X2     49U
#define BUCK_ADC_CONVERSION_CYCLES_X2 25U
#define BUCK_ADC_SEQUENCE_CYCLES_X2 \
    ((BUCK_ADC_SAMPLE_CYCLES_X2 + BUCK_ADC_CONVERSION_CYCLES_X2) \
     * BUCK_ADC_REGULAR_RANK_COUNT)

#endif
