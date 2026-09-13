/**
 * @file pid.h
 * @brief Discrete PID controller with integral clamping and output saturation
 *
 * Pure C with no RTOS or hardware dependencies, so it can run in host tests
 * and be instantiated several times (e.g. an angle loop and a velocity loop).
 */

#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef struct {
    /* Configuration */
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    float integral_limit;   /**< Clamp on the accumulated error (anti-windup) */
    float output_limit;     /**< Clamp on the output, symmetric */

    /* State */
    float integral;         /**< Accumulated error x dt */
    float prev_error;       /**< Error of the previous update */

    /* Last update, for telemetry */
    float p_term;
    float i_term;
    float d_term;
    float output;
} PID_t;

/** Clear the integral, derivative history and last terms; gains are kept */
void pid_reset(PID_t *pid);

/**
 * @brief Run one controller update
 *
 * @param pid    Controller
 * @param error  Setpoint minus measurement
 * @param dt     Time since the previous update in seconds, > 0
 * @return Output clamped to +/- output_limit
 */
float pid_update(PID_t *pid, float error, float dt);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CONTROL_PID_H
