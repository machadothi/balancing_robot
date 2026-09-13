/**
 * @file mixer.c
 * @brief Differential drive mixing with saturation and deadband compensation
 */

#include <math.h>

#include "control/mixer.h"

static int16_t mixer_wheel(float value, int16_t limit, int16_t deadband) {
    /* Saturate before narrowing: the turn term can push |value| past the limit */
    int16_t magnitude = (int16_t)fminf(fabsf(value), (float)limit);

    if (magnitude > 0 && magnitude < deadband) {
        magnitude = deadband;
    }
    return (value < 0.0f) ? (int16_t)-magnitude : magnitude;
}

Mixer_Output_t mixer_mix(float output, float turn, int16_t limit, int16_t deadband) {
    Mixer_Output_t wheels = {
        .left = mixer_wheel(output + turn, limit, deadband),
        .right = mixer_wheel(output - turn, limit, deadband),
    };
    return wheels;
}
