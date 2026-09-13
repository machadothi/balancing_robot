/**
 * @file pid.c
 * @brief Discrete PID controller
 */

#include "control/pid.h"

static float clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void pid_reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->output = 0.0f;
}

float pid_update(PID_t *pid, float error, float dt) {
    pid->p_term = pid->kp * error;

    /* Clamping the accumulated error bounds the I term at ki * integral_limit */
    pid->integral = clamp(pid->integral + error * dt, pid->integral_limit);
    pid->i_term = pid->ki * pid->integral;

    /* Derivative on error: equals -kd * d(measurement)/dt while the setpoint is constant */
    pid->d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    pid->output = clamp(pid->p_term + pid->i_term + pid->d_term, pid->output_limit);
    return pid->output;
}
