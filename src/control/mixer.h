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
 * +/- limit, then compensated for the motor's dead zone continuously:
 * magnitudes 1..limit map linearly onto deadband..limit, so the smallest
 * command already turns the wheel and the mapping has no jump. Below one count
 * the wheel stays stopped.
 *
 * @param deadband_left   Smallest command that keeps the left wheel turning
 * @param deadband_right  Same for the right wheel
 */
Mixer_Output_t mixer_mix(float output, float turn, int16_t limit,
                         int16_t deadband_left, int16_t deadband_right);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CONTROL_MIXER_H
