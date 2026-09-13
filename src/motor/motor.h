/**
 * @file motor.h
 * @brief TB6612FNG Motor Driver Interface
 * 
 * Hardware abstraction for dual DC motor control using the TB6612FNG
 * H-bridge driver IC. Provides direction control, PWM speed control,
 * brake/coast modes, and encoder feedback.
 * 
 * Pin Mapping:
 * 
 *   TB6612FNG    STM32       Function
 *   ---------    -----       --------
 *   STBY         PB4         Standby control (active-high to enable)
 *   AIN1         PB3         Motor A direction input 1
 *   AIN2         PA8         Motor A direction input 2
 *   BIN1         PB5         Motor B direction input 1
 *   BIN2         PB8         Motor B direction input 2
 *   PWMA         PB1         Motor A speed (TIM3_CH4)
 *   PWMB         PB0         Motor B speed (TIM3_CH3)
 * 
 *   Motor A Encoder  PA5     Rising edge interrupt
 *   Motor B Encoder  PA6     Rising edge interrupt
 * 
 * Direction Truth Table (TB6612FNG):
 * 
 *   IN1  IN2  Mode
 *   ---  ---  ----
 *   H    L    Forward (CW)
 *   L    H    Reverse (CCW)
 *   H    H    Short brake
 *   L    L    Coast (free run)
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdbool.h>
#include <stdint.h>
#include <libopencm3/stm32/gpio.h>

/* ==========================================================================
 * Pin Definitions
 * ========================================================================== */

/** @defgroup TB6612_Pins TB6612FNG Pin Mapping
 *  @{
 */
#define TB6612_STBY     GPIO4   /**< Standby pin (Port B) - Active HIGH to enable */
#define TB6612_AIN1     GPIO3   /**< Motor A direction input 1 (Port B) */
#define TB6612_AIN2     GPIO8   /**< Motor A direction input 2 (Port A) */
#define TB6612_BIN1     GPIO5   /**< Motor B direction input 1 (Port B) */
#define TB6612_BIN2     GPIO8   /**< Motor B direction input 2 (Port B) */
#define TB6612_PWMA     GPIO1   /**< Motor A PWM (Port B, TIM3_CH4) */
#define TB6612_PWMB     GPIO0   /**< Motor B PWM (Port B, TIM3_CH3) */
/** @} */

/** @defgroup Encoder_Pins Encoder Input Pins
 *  @{
 */
#define MOTOR1_ENCODER  GPIO5   /**< Motor A encoder input (Port A, EXTI5) */
#define MOTOR2_ENCODER  GPIO6   /**< Motor B encoder input (Port A, EXTI6) */
/** @} */

/* ==========================================================================
 * Initialization
 * ========================================================================== */

/**
 * @brief Initialize motor driver hardware
 * 
 * Configures GPIO pins, PWM timer, and encoder interrupts.
 * Motors start in stopped state (speed = 0) with driver enabled.
 */
void motor_init(void);

/**
 * @brief Deinitialize motor driver
 * 
 * Puts the TB6612FNG into standby mode and releases PWM resources.
 */
void motor_deinit(void);

/* ==========================================================================
 * Direction Control
 * ========================================================================== */

/**
 * @brief Set Motor A (motor1) direction
 * @param direction  true = forward (CW), false = reverse (CCW)
 */
void motor1_set_direction(bool direction);

/**
 * @brief Set Motor B (motor2) direction
 * @param direction  true = forward (CW), false = reverse (CCW)
 */
void motor2_set_direction(bool direction);

/* ==========================================================================
 * Speed Control
 * ========================================================================== */

/**
 * @brief Set Motor A speed
 * @param speed  PWM duty cycle (0 = stopped, 255 = full speed)
 */
void motor1_set_speed(uint8_t speed);

/**
 * @brief Set Motor B speed
 * @param speed  PWM duty cycle (0 = stopped, 255 = full speed)
 */
void motor2_set_speed(uint8_t speed);

/* ==========================================================================
 * Brake and Coast Modes
 * ========================================================================== */

/**
 * @brief Apply short brake to Motor A
 */
void motor1_brake(void);

/**
 * @brief Apply short brake to Motor B
 */
void motor2_brake(void);

/**
 * @brief Set Motor A to coast (free run)
 */
void motor1_coast(void);

/**
 * @brief Set Motor B to coast (free run)
 */
void motor2_coast(void);

/* ==========================================================================
 * Standby Control
 * ========================================================================== */

/**
 * @brief Enable or disable motor driver standby mode
 * @param enable  true = enter standby, false = exit standby
 */
void motor_standby(bool enable);

/**
 * @brief Cut all motor drive immediately
 *
 * Safe from fault handlers and with interrupts disabled: register writes
 * only, no RTOS calls. Harmless before motor_init().
 */
void motor_emergency_stop(void);

/* ==========================================================================
 * Encoder Functions
 * ========================================================================== */

/**
 * @brief Get Motor A encoder count
 * @return Encoder pulse count
 */
uint32_t motor1_get_encoder(void);

/**
 * @brief Get Motor B encoder count
 * @return Encoder pulse count
 */
uint32_t motor2_get_encoder(void);

/**
 * @brief Reset Motor A encoder count to zero
 */
void motor1_reset_encoder(void);

/**
 * @brief Reset Motor B encoder count to zero
 */
void motor2_reset_encoder(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MOTOR_H
