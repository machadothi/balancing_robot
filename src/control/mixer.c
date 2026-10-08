/**
 * @file mixer.c
 * @brief Differential drive mixing with saturation and deadband compensation
 */

#include <math.h>

#include "control/mixer.h"

static int16_t mixer_wheel(float value, int16_t limit, int16_t deadband) {
    /* Saturate first: the turn term can push |value| past the limit */
    float magnitude = fminf(fabsf(value), (float)limit);

    if (magnitude < 1.0f) {
        return 0;
    }

    /* 1..limit -> deadband..limit: continuous and monotonic, unlike raising
     * small commands to the deadband, which made the output jump near zero */
    int16_t mapped = (int16_t)(deadband + magnitude * (float)(limit - deadband) / (float)limit);
    return (value < 0.0f) ? (int16_t)-mapped : mapped;
}

Mixer_Output_t mixer_mix(float output, float turn, int16_t limit,
                         int16_t deadband_left, int16_t deadband_right) {
    Mixer_Output_t wheels = {
        .left = mixer_wheel(output + turn, limit, deadband_left),
        .right = mixer_wheel(output - turn, limit, deadband_right),
    };
    return wheels;
}
