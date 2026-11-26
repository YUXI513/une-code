#ifndef BUCK_DISPLAY_FILTER_H
#define BUCK_DISPLAY_FILTER_H

#include <math.h>
#include <stdint.h>

/* OLED/Watch display only. Never use this slow value for PWM or PI feedback.
 * Nominal samples arrive every 10 ms; clear after a missing 30 ms window.
 * Time constant 200 ms: a voltage step takes about 0.6 s to settle to 95%. */
#define BUCK_DISPLAY_FILTER_TAU_MS 200.0f
#define BUCK_DISPLAY_STALE_MS      30U
#define BUCK_DISPLAY_DIGIT_CONFIRM_MS 800U
#define BUCK_DISPLAY_DIGIT_FAST_CHANGE 3

typedef struct
{
    float value;
    uint32_t last_ms;
    uint32_t ready;
    uint32_t source;
} BuckDisplayFilter;

static inline void BuckDisplayFilter_Reset(BuckDisplayFilter *filter)
{
    filter->value = NAN;
    filter->ready = 0U;
}

/* source distinguishes confirmed values from nominal-reference estimates.
 * Seed directly after startup, an invalid reading, a gap or source change. */
static inline float BuckDisplayFilter_Update(BuckDisplayFilter *filter,
                                              float sample, uint32_t now,
                                              uint32_t source)
{
    if (!isfinite(sample))
    {
        BuckDisplayFilter_Reset(filter);
        return NAN;
    }
    uint32_t elapsed = now - filter->last_ms;
    if (filter->ready == 0U || elapsed > BUCK_DISPLAY_STALE_MS || source != filter->source)
        filter->value = sample;
    else if (elapsed != 0U)
    {
        float alpha = (float)elapsed / (BUCK_DISPLAY_FILTER_TAU_MS + (float)elapsed);
        filter->value += alpha * (sample - filter->value);
    }
    filter->last_ms = now;
    filter->source = source;
    filter->ready = 1U;
    return filter->value;
}

static inline float BuckDisplayFilter_Get(BuckDisplayFilter *filter, uint32_t now)
{
    if (filter->ready == 0U || (uint32_t)(now - filter->last_ms) > BUCK_DISPLAY_STALE_MS)
    {
        BuckDisplayFilter_Reset(filter);
        return NAN;
    }
    return filter->value;
}

/* Stabilize only the OLED's last digit. One/two centivolt changes need a
 * consistent candidate for 800 ms; >= 0.03 V follows the existing filter.
 * Caller supplies fresh filtered values. NAN immediately clears history. */
typedef struct
{
    int32_t shown;
    int32_t pending;
    uint32_t pending_since;
    uint32_t ready;
    uint32_t source;
} BuckDisplayDigits;

static inline float BuckDisplayDigits_Update(BuckDisplayDigits *digits,
                                              float voltage, uint32_t now,
                                              uint32_t source)
{
    /* ADC/divider-derived voltages are bounded well inside int32 range. */
    if (!isfinite(voltage) || fabsf(voltage) > 1000000.0f)
    {
        digits->ready = 0U;
        return NAN;
    }
    int32_t candidate = (int32_t)lroundf(voltage * 100.0f);
    if (digits->ready == 0U || source != digits->source)
    {
        digits->shown = candidate;
        digits->pending = candidate;
        digits->pending_since = now;
        digits->source = source;
        digits->ready = 1U;
    }
    else
    {
        int32_t change = candidate - digits->shown;
        if (change >= BUCK_DISPLAY_DIGIT_FAST_CHANGE || change <= -BUCK_DISPLAY_DIGIT_FAST_CHANGE)
        {
            digits->shown = candidate;
            digits->pending = candidate;
            digits->pending_since = now;
        }
        else if (candidate == digits->shown || candidate != digits->pending)
        {
            digits->pending = candidate;
            digits->pending_since = now;
        }
        else if ((uint32_t)(now - digits->pending_since) >= BUCK_DISPLAY_DIGIT_CONFIRM_MS)
            digits->shown = candidate;
    }
    return (float)digits->shown * 0.01f;
}

#endif
