/**
 * @file fmt.c
 * @brief Number formatting without float printf
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "util/fmt.h"

const char *fmt_fixed(char *buf, size_t len, float value, int decimals) {
    static const int32_t scales[] = { 1, 10, 100, 1000, 10000 };

    if (decimals < 0) {
        decimals = 0;
    } else if (decimals > 4) {
        decimals = 4;
    }

    float scaled = value * (float)scales[decimals];
    if (!isfinite(scaled) || fabsf(scaled) > 2.0e9f) {
        snprintf(buf, len, "%s", isnan(value) ? "nan" : "ovf");
        return buf;
    }

    int32_t fixed = (int32_t)lroundf(scaled);
    uint32_t magnitude = (fixed < 0) ? (uint32_t)(-(int64_t)fixed) : (uint32_t)fixed;
    const char *sign = (fixed < 0) ? "-" : "";
    uint32_t scale = (uint32_t)scales[decimals];

    if (decimals == 0) {
        snprintf(buf, len, "%s%lu", sign, (unsigned long)magnitude);
    } else {
        snprintf(buf, len, "%s%lu.%0*lu", sign, (unsigned long)(magnitude / scale),
                 decimals, (unsigned long)(magnitude % scale));
    }
    return buf;
}
