/**
 * @file motor.h
 * @brief TB6612FNG motor driver interface
 * 
 * Provides control for dual DC motors using the TB6612FNG H-bridge driver.
 * PWM control via TIM3 channels 3 and 4.
 * 
 * Pin mapping:
 * - STBY:  PB4  (Standby control)
 * - AIN1:  PB3  (Motor A direction 1)
 * - AIN2:  PA8  (Motor A direction 2)
 * - BIN1:  PB5  (Motor B direction 1)
 * - BIN2:  PB8  (Motor B direction 2)
 * - PWMA:  PB1  (Motor A speed - TIM3_CH4)
 * - PWMB:  PB0  (Motor B speed - TIM3_CH3)
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <libopencm3/stm32/gpio.h>

/* ==========================================================================
 * Pin Definitions
 * ========================================================================== */

/** @defgroup Motor_Pins Motor Driver Pin Definitions
 *  @{
 */
#define TB6612_STBY     GPIO4   /**< Standby pin (Port B) */
#define TB6612_AIN1     GPIO3   /**< Motor A IN1 (Port B) */
#define TB6612_AIN2     GPIO8   /**< Motor A IN2 (Port A) */
#define TB6612_BIN1     GPIO5   /**< Motor B IN1 (Port B) */
#define TB6612_BIN2     GPIO8   /**< Motor B IN2 (Port B) */
#define TB6612_PWMA     GPIO1   /**< Motor A PWM (Port B) */
#define TB6612_PWMB     GPIO0   /**< Motor B PWM (Port B) */
/** @} */

/** @defgroup Encoder_Pins Encoder Pin Definitions
 *  @{
 */
#define MOTOR1_ENCODER  GPIO5   /**< Motor 1 encoder (Port A) */
#define MOTOR2_ENCODER  GPIO6   /**< Motor 2 encoder (Port A) */
/** @} */

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize motor driver hardware
 * 
 * Configures GPIO pins and PWM timer for motor control.
 * Motors are initially stopped (speed = 0).
 */
void motor_init(void);

/**
 * @brief Set motor 1 direction
 * @param direction true = forward, false = reverse
 */
void motor1_set_direction(bool direction);

/**
 * @brief Set motor 2 direction
 * @param direction true = forward, false = reverse
 */
void motor2_set_direction(bool direction);

/**
 * @brief Set motor 1 speed
 * @param speed PWM duty cycle (0-255, where 255 = 100%)
 */
void motor1_set_speed(uint8_t speed);

/**
 * @brief Set motor 2 speed
 * @param speed PWM duty cycle (0-255, where 255 = 100%)
 */
void motor2_set_speed(uint8_t speed);

/**
 * @brief Motor demonstration task
 * 
 * FreeRTOS task that cycles through motor speeds for testing.
 * 
 * @param args Task arguments (unused)
 */
void motor_demo_task(void *args);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
