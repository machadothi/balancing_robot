/**
 * @file fmt.h
 * @brief Number formatting without float printf
 */

#ifndef UTIL_FMT_H
#define UTIL_FMT_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * @brief Format a float with fixed decimals using integer printf only
 *
 * Float printf needs far more stack. Rounds once on the scaled value, so the
 * sign of small negatives and carries (1.96 -> "2.0") are correct. Prints
 * "nan" or "ovf" for values that do not fit.
 *
 * @param buf       Output buffer (16 bytes is enough)
 * @param len       Buffer size
 * @param value     Value to format
 * @param decimals  Digits after the point, 0 to 4
 * @return buf
 */
const char *fmt_fixed(char *buf, size_t len, float value, int decimals);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // UTIL_FMT_H
