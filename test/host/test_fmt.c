/**
 * @file test_fmt.c
 * @brief fmt_fixed(): the number format of every AT reply and telemetry line
 */

#include "util/fmt.h"
#include "unit.h"

static const char *fmt(float value, int decimals) {
    static char buf[32];
    const char *result = fmt_fixed(buf, sizeof(buf), value, decimals);
    CHECK(result == buf);
    return buf;
}

int main(void) {
    CHECK_STR(fmt(0.0f, 2), "0.00");
    CHECK_STR(fmt(-0.5f, 2), "-0.50");
    CHECK_STR(fmt(-0.004f, 2), "0.00");        /* no "-0.00" */
    CHECK_STR(fmt(1.996f, 2), "2.00");         /* carry into the integer part */
    CHECK_STR(fmt(-99.999f, 2), "-100.00");
    CHECK_STR(fmt(12.3456f, 4), "12.3456");
    CHECK_STR(fmt(30.0f, 1), "30.0");
    CHECK_STR(fmt(-30.0f, 1), "-30.0");
    CHECK_STR(fmt(0.125f, 3), "0.125");
    CHECK_STR(fmt(-0.998f, 3), "-0.998");
    return UNIT_RESULT();
}
