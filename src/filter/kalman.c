/**
 * @file kalman.c
 * @brief Kalman filter implementation for 1D angle estimation
 * 
 * Implements a simplified 1D Kalman filter optimized for IMU sensor fusion.
 * Combines gyroscope (angular rate) and accelerometer (absolute angle)
 * to produce a smooth, accurate angle estimate.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include "config.h"
#include "filter/filter.h"

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void kalman_init(KalmanFilter_t *kf) {
    kf->angle = 0.0f;
    kf->uncertainty = 1.0f;
    kf->Q_angle = KALMAN_Q_ANGLE;
    kf->R_measure = KALMAN_R_MEASURE;
}

void kalman_init_custom(KalmanFilter_t *kf, float Q_angle, float R_measure) {
    kf->angle = 0.0f;
    kf->uncertainty = 1.0f;
    kf->Q_angle = Q_angle;
    kf->R_measure = R_measure;
}

float kalman_update(KalmanFilter_t *kf, float gyro_rate, float acc_angle, float dt) {
    /* PREDICT: Integrate gyroscope to estimate new angle */
    kf->angle += gyro_rate * dt;
    kf->uncertainty += kf->Q_angle;
    
    /* UPDATE: Correct prediction with accelerometer measurement */
    float K = kf->uncertainty / (kf->uncertainty + kf->R_measure);
    kf->angle += K * (acc_angle - kf->angle);
    kf->uncertainty *= (1.0f - K);
    
    return kf->angle;
}

/* ==========================================================================
 * AttitudeFilter_t interface
 * ========================================================================== */

static void kalman_seed_state(void *state, float angle) {
    ((KalmanFilter_t *)state)->angle = angle;
}

static float kalman_update_state(void *state, float gyro_rate, float acc_angle, float dt) {
    return kalman_update((KalmanFilter_t *)state, gyro_rate, acc_angle, dt);
}

AttitudeFilter_t kalman_filter_interface(KalmanFilter_t *kf) {
    AttitudeFilter_t filter = {
        .name = "kalman",
        .state = kf,
        .seed = kalman_seed_state,
        .update = kalman_update_state,
    };
    return filter;
}
