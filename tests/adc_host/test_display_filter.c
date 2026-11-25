/* Native test: gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc
 * tests/adc_host/test_display_filter.c -lm -o display_filter_test */
#include "buck_display_filter.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    BuckDisplayFilter filter = {0};
    uint32_t now = 0U;
    assert(isnan(BuckDisplayFilter_Get(&filter, now)));
    assert(BuckDisplayFilter_Update(&filter, 15.8f, now, 1U) == 15.8f);
    float minimum = 100.0f, maximum = 0.0f;
    for (uint32_t sample = 0U; sample < 200U; ++sample)
    {
        now += 10U;
        float value = BuckDisplayFilter_Update(&filter, sample & 1U ? 15.95f : 15.75f, now, 1U);
        if (sample >= 100U)
        {
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
        }
    }
    assert(maximum - minimum < 0.02f);
    for (uint32_t sample = 0U; sample < 60U; ++sample)
    {
        now += 10U;
        (void)BuckDisplayFilter_Update(&filter, 10.0f, now, 1U);
    }
    assert(fabsf(BuckDisplayFilter_Get(&filter, now) - 10.0f) < 0.4f);
    /* A fault must immediately clear the value and reset subsequent startup. */
    assert(isnan(BuckDisplayFilter_Update(&filter, NAN, now + 10U, 1U)));
    now += 20U;
    assert(BuckDisplayFilter_Update(&filter, 25.0f, now, 1U) == 25.0f);
    /* Changing from estimate to confirmed data must not retain the old scale. */
    now += 10U;
    assert(BuckDisplayFilter_Update(&filter, 20.0f, now, 0U) == 20.0f);
    now += 31U;
    assert(isnan(BuckDisplayFilter_Get(&filter, now)));
    assert(BuckDisplayFilter_Update(&filter, 6.4f, now, 0U) == 6.4f);
    /* Unsigned tick wrap is a short interval, not a stale gap. */
    filter.last_ms = UINT32_MAX - 4U;
    filter.value = 10.0f;
    filter.ready = 1U;
    float wrapped = BuckDisplayFilter_Update(&filter, 20.0f, 5U, 0U);
    assert(wrapped > 10.0f && wrapped < 11.0f);
    float unchanged = BuckDisplayFilter_Update(&filter, 20.0f, 5U, 0U);
    assert(unchanged == wrapped);
    puts("PASS display noise reduction, step response, faults, gaps, source change and tick wrap");

    BuckDisplayDigits digits = {0};
    now = 0U;
    assert(fabsf(BuckDisplayDigits_Update(&digits, 15.86f, now, 1U) - 15.86f) < 0.0001f);
    for (uint32_t sample = 0U; sample < 30U; ++sample)
    {
        now += 200U;
        float candidate = sample % 3U == 0U ? 15.85f : (sample % 3U == 1U ? 15.86f : 15.87f);
        assert(fabsf(BuckDisplayDigits_Update(&digits, candidate, now, 1U) - 15.86f) < 0.0001f);
    }
    /* A real small DC change must eventually appear, in both directions. */
    for (uint32_t sample = 0U; sample < 6U; ++sample)
    {
        now += 200U;
        (void)BuckDisplayDigits_Update(&digits, 15.87f, now, 1U);
    }
    assert(fabsf(BuckDisplayDigits_Update(&digits, 15.87f, now, 1U) - 15.87f) < 0.0001f);
    for (uint32_t sample = 0U; sample < 6U; ++sample)
    {
        now += 200U;
        (void)BuckDisplayDigits_Update(&digits, 15.85f, now, 1U);
    }
    assert(fabsf(BuckDisplayDigits_Update(&digits, 15.85f, now, 1U) - 15.85f) < 0.0001f);
    /* Large steps and source changes do not wait for digit confirmation. */
    assert(fabsf(BuckDisplayDigits_Update(&digits, 20.0f, now + 200U, 1U) - 20.0f) < 0.0001f);
    assert(fabsf(BuckDisplayDigits_Update(&digits, 20.01f, now + 400U, 0U) - 20.01f) < 0.0001f);
    assert(isnan(BuckDisplayDigits_Update(&digits, NAN, now + 600U, 0U)));
    assert(fabsf(BuckDisplayDigits_Update(&digits, 6.4f, now + 800U, 0U) - 6.4f) < 0.0001f);
    /* Persistence timing must work across the HAL tick wrap too. */
    (void)BuckDisplayDigits_Update(&digits, NAN, UINT32_MAX - 500U, 1U);
    (void)BuckDisplayDigits_Update(&digits, 10.0f, UINT32_MAX - 400U, 1U);
    (void)BuckDisplayDigits_Update(&digits, 10.01f, UINT32_MAX - 300U, 1U);
    assert(fabsf(BuckDisplayDigits_Update(&digits, 10.01f, 600U, 1U) - 10.01f) < 0.0001f);
    puts("PASS digit stability, sustained small changes, large steps, invalidation, source change and wrap");
    return 0;
}
