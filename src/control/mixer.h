/**
 * @file mixer.h
 * @brief Differential drive mixing with saturation and deadband compensation
 *
 * Pure C, no hardware dependencies.
 */

#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
    int16_t left;   /**< Signed wheel command, sign = direction */
    int16_t right;
} Mixer_Output_t;

/**
 * @brief Turn a balance output and a turn rate into two wheel commands
 *
 * left = output + turn, right = output - turn. Each wheel is saturated to
 * +/- limit, and any non-zero magnitude below `deadband` is raised to it so
 * the motors overcome static friction.
 */
Mixer_Output_t mixer_mix(float output, float turn, int16_t limit, int16_t deadband);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CONTROL_MIXER_H
