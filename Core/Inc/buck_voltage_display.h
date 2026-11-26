#ifndef BUCK_VOLTAGE_DISPLAY_H
#define BUCK_VOLTAGE_DISPLAY_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* Signed diagnostic text only. No filtering, calibration or control logic. */
static inline void BuckVoltageDisplay_Format(char *line, size_t size,
                                             const char *label, float voltage,
                                             uint32_t estimate)
{
    if (!isfinite(voltage) || fabsf(voltage) > 1000000.0f)
    {
        snprintf(line, size, "%s:--.--V", label);
        return;
    }
    int32_t signed_hundredths = (int32_t)lroundf(voltage * 100.0f);
    unsigned long hundredths = (unsigned long)(signed_hundredths < 0
        ? -signed_hundredths : signed_hundredths);
    snprintf(line, size, "%s:%s%s%lu.%02luV", label, estimate ? "~" : "",
             signed_hundredths < 0 ? "-" : "", hundredths / 100UL,
             hundredths % 100UL);
}

#endif
