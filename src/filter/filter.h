/**
 * @file filter.h
 * @brief Signal processing filters for IMU sensor fusion
 * 
 * Provides two complementary filtering approaches:
 * - **Kalman Filter**: Optimal linear estimator with adaptive gain
 * - **Complementary Filter**: Simple and computationally efficient
 * 
 * Both filters combine gyroscope (fast, drifts) and accelerometer 
 * (slow, noisy) data to produce a stable angle estimate.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Kalman Filter
 * ========================================================================== */

/**
 * @brief Kalman filter state structure
 */
typedef struct {
    float angle;        /**< Current angle estimate (degrees) */
    float uncertainty;  /**< Estimation uncertainty (variance) */
    float Q_angle;      /**< Process noise - trust in gyro integration */
    float R_measure;    /**< Measurement noise - trust in accelerometer */
} KalmanFilter_t;

/**
 * @brief Initialize Kalman filter with default parameters
 * 
 * Default values (tuned for MPU6050):
 * - Q_angle = 0.1 (from config.h)
 * - R_measure = 0.5 (from config.h)
 * 
 * @param kf Pointer to Kalman filter structure
 */
void kalman_init(KalmanFilter_t *kf);

/**
 * @brief Initialize Kalman filter with custom parameters
 * 
 * @param kf Pointer to Kalman filter structure
 * @param Q_angle Process noise (higher = trust gyro more, noisier output)
 * @param R_measure Measurement noise (higher = trust accel less, smoother)
 */
void kalman_init_custom(KalmanFilter_t *kf, float Q_angle, float R_measure);

/**
 * @brief Update Kalman filter with new sensor readings
 * 
 * @param kf Pointer to Kalman filter structure
 * @param gyro_rate Gyroscope angular rate (degrees/second)
 * @param acc_angle Angle calculated from accelerometer (degrees)
 * @param dt Time step since last update (seconds)
 * @return Filtered angle estimate (degrees)
 */
float kalman_update(KalmanFilter_t *kf, float gyro_rate, float acc_angle, float dt);

/* ==========================================================================
 * Complementary Filter
 * ========================================================================== */

/**
 * @brief Complementary filter state structure
 */
typedef struct {
    float angle;  /**< Current angle estimate (degrees) */
    float alpha;  /**< Filter coefficient (0-1, higher = trust gyro more) */
} ComplementaryFilter_t;

/**
 * @brief Initialize Complementary filter with default alpha
 * 
 * Default alpha = 0.96 (from config.h), meaning:
 * - 96% weight on gyroscope integration
 * - 4% weight on accelerometer absolute reference
 * 
 * @param cf Pointer to Complementary filter structure
 */
void complementary_init(ComplementaryFilter_t *cf);

/**
 * @brief Initialize Complementary filter with custom alpha
 * 
 * @param cf Pointer to Complementary filter structure
 * @param alpha Filter coefficient (typical range: 0.90-0.99)
 *              Higher = smoother but may drift
 *              Lower = more responsive but noisier
 */
void complementary_init_custom(ComplementaryFilter_t *cf, float alpha);

/**
 * @brief Update Complementary filter with new sensor readings
 * 
 * Implements: angle = alpha*(angle + gyro*dt) + (1-alpha)*acc_angle
 * 
 * @param cf Pointer to Complementary filter structure
 * @param gyro_rate Gyroscope angular rate (degrees/second)
 * @param acc_angle Angle calculated from accelerometer (degrees)
 * @param dt Time step since last update (seconds)
 * @return Filtered angle estimate (degrees)
 */
float complementary_update(ComplementaryFilter_t *cf, float gyro_rate, float acc_angle, float dt);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // FILTER_H
