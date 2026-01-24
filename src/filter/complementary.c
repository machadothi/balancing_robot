/**
 * @file complementary.c
 * @brief Complementary filter implementation for 1D angle estimation
 * 
 * Implements a simple yet effective sensor fusion filter:
 * - High-pass filter on gyroscope (removes drift)
 * - Low-pass filter on accelerometer (removes vibration noise)
 * 
 * Formula: angle = alpha*(angle + gyro*dt) + (1-alpha)*acc_angle
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include "config.h"
#include "filter/filter.h"

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void complementary_init(ComplementaryFilter_t *cf) {
    cf->angle = 0.0f;
    cf->alpha = COMPLEMENTARY_ALPHA;
}

void complementary_init_custom(ComplementaryFilter_t *cf, float alpha) {
    cf->angle = 0.0f;
    cf->alpha = alpha;
}

float complementary_update(ComplementaryFilter_t *cf, float gyro_rate, float acc_angle, float dt) {
    /* Combine high-pass filtered gyro with low-pass filtered accelerometer */
    cf->angle = cf->alpha * (cf->angle + gyro_rate * dt) + (1.0f - cf->alpha) * acc_angle;
    return cf->angle;
}
