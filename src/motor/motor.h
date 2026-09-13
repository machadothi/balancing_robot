/**
 * @file motor.h
 * @brief Two-wheel motor interface, implemented once per board
 *
 * Implementations: motor.c (Blue Pill + TB6612FNG), motor_hiwonder.c
 * (Hiwonder F407 board). The build selects one with MOTOR_SOURCE.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_COUNT
} Motor_Id_t;

/** Full-scale magnitude of a motor command (100 % duty) */
#define MOTOR_COMMAND_MAX   255

/**
 * @brief Configure pins, PWM and encoders
 *
 * Motors start stopped with the driver enabled; callers usually follow with
 * motor_standby(true).
 */
void motor_init(void);

/**
 * @brief Drive one wheel
 *
 * @param id       Wheel
 * @param command  -MOTOR_COMMAND_MAX..MOTOR_COMMAND_MAX; the sign selects the
 *                 direction (positive = forward), the magnitude the duty cycle.
 *                 Out-of-range values are clamped.
 */
void motor_set(Motor_Id_t id, int16_t command);

/** Short-brake one wheel (both bridge inputs high) */
void motor_brake(Motor_Id_t id);

/** Let one wheel coast (both bridge inputs low) */
void motor_coast(Motor_Id_t id);

/**
 * @brief Enter or leave driver standby
 *
 * In standby no wheel is driven, whatever the last command was.
 */
void motor_standby(bool enable);

/**
 * @brief Cut all motor drive immediately
 *
 * Safe from fault handlers and with interrupts disabled: register writes
 * only, no RTOS calls. Harmless before motor_init().
 */
void motor_emergency_stop(void);

/**
 * @brief Encoder count of one wheel
 *
 * Quadrature boards count up and down; the Blue Pill counts rising edges only.
 */
int32_t motor_get_encoder(Motor_Id_t id);

/** Reset one wheel's encoder count to zero */
void motor_reset_encoder(Motor_Id_t id);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MOTOR_H
