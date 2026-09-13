/**
 * @file unit.h
 * @brief Minimal checks for the host tests: report every failure, keep going
 */
#ifndef UNIT_H
#define UNIT_H

#include <math.h>
#include <stdio.h>
#include <string.h>

static int unit_failures = 0;

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            unit_failures++;                                                    \
            printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);     \
        }                                                                       \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                 \
    do {                                                                        \
        double a_ = (actual), e_ = (expected);                                  \
        if (!(fabs(a_ - e_) <= (tolerance))) {                                  \
            unit_failures++;                                                    \
            printf("%s:%d: %s = %g, expected %g +- %g\n", __FILE__, __LINE__,   \
                   #actual, a_, e_, (double)(tolerance));                       \
        }                                                                       \
    } while (0)

#define CHECK_STR(actual, expected)                                             \
    do {                                                                        \
        const char *a_ = (actual), *e_ = (expected);                            \
        if (strcmp(a_, e_) != 0) {                                              \
            unit_failures++;                                                    \
            printf("%s:%d: %s = \"%s\", expected \"%s\"\n", __FILE__, __LINE__, \
                   #actual, a_, e_);                                            \
        }                                                                       \
    } while (0)

/** Print the summary; use as `return UNIT_RESULT();` from main */
#define UNIT_RESULT()                                                           \
    (printf("%s: %s (%d failures)\n", __FILE__,                                 \
            unit_failures ? "FAILED" : "passed", unit_failures),                \
     unit_failures != 0)

#endif // UNIT_H
