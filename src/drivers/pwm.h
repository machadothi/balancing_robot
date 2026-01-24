/**
 * @file pwm.h
 * @brief PWM (Pulse Width Modulation) Driver Interface
 * 
 * Hardware abstraction layer for STM32F103 timer-based PWM generation.
 * Supports multiple independent PWM channels with configurable frequency
 * and duty cycle.
 * 
 * Features:
 *   - Configurable PWM frequency (1Hz - 100kHz typical)
 *   - 16-bit duty cycle resolution
 *   - Support for TIM2, TIM3, TIM4 general-purpose timers
 *   - Multiple output modes (push-pull, open-drain)
 *   - Dead-time insertion (for H-bridge applications)
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief PWM operation result codes
 */
typedef enum {
    PWM_OK = 0,             /**< Operation successful */
    PWM_INVALID_TIMER,      /**< Invalid timer specified */
    PWM_INVALID_CHANNEL,    /**< Invalid channel for this timer */
    PWM_INVALID_FREQUENCY,  /**< Frequency out of range */
    PWM_NOT_INITIALIZED,    /**< PWM not initialized */
} PWM_Status_t;

/**
 * @brief PWM output mode
 */
typedef enum {
    PWM_OUTPUT_PUSH_PULL,   /**< Standard push-pull output */
    PWM_OUTPUT_OPEN_DRAIN,  /**< Open-drain output (for external pull-up) */
} PWM_OutputMode_t;

/**
 * @brief PWM channel identifier
 */
typedef enum {
    PWM_CHANNEL_1 = 1,
    PWM_CHANNEL_2 = 2,
    PWM_CHANNEL_3 = 3,
    PWM_CHANNEL_4 = 4,
} PWM_Channel_t;

/**
 * @brief PWM timer configuration
 */
typedef struct {
    uint32_t timer;             /**< Timer peripheral (TIM2, TIM3, TIM4) */
    uint32_t frequency_hz;      /**< PWM frequency in Hz */
    uint16_t resolution;        /**< PWM resolution (period value, e.g., 1000 for 0.1% steps) */
    bool     initialized;       /**< Initialization flag */
} PWM_Timer_t;

/**
 * @brief PWM channel configuration
 */
typedef struct {
    PWM_Timer_t     *timer;     /**< Pointer to parent timer */
    PWM_Channel_t   channel;    /**< Channel number (1-4) */
    uint32_t        gpio_port;  /**< GPIO port for output pin */
    uint16_t        gpio_pin;   /**< GPIO pin for output */
    PWM_OutputMode_t mode;      /**< Output mode */
    uint16_t        duty;       /**< Current duty cycle (0 to timer->resolution) */
    bool            enabled;    /**< Channel enable state */
} PWM_Channel_Config_t;

/* ==========================================================================
 * Timer Initialization
 * ========================================================================== */

/**
 * @brief Initialize PWM timer
 * 
 * Configures the timer peripheral for PWM generation. Must be called
 * before configuring any channels on this timer.
 * 
 * @param pwm           Pointer to timer configuration structure
 * @param timer         Timer peripheral (TIM2, TIM3, or TIM4)
 * @param frequency_hz  Desired PWM frequency in Hz
 * @param resolution    PWM resolution (period value, e.g., 1000)
 * @return PWM_OK on success, error code otherwise
 * 
 * @note The actual frequency depends on the system clock and may differ
 *       slightly from the requested value.
 * 
 * Example:
 * @code
 *     PWM_Timer_t motor_pwm;
 *     pwm_timer_init(&motor_pwm, TIM3, 20000, 1000);  // 20kHz, 0.1% resolution
 * @endcode
 */
PWM_Status_t pwm_timer_init(PWM_Timer_t *pwm, uint32_t timer, 
                            uint32_t frequency_hz, uint16_t resolution);

/**
 * @brief Deinitialize PWM timer
 * 
 * Stops the timer and releases resources.
 * 
 * @param pwm   Pointer to timer configuration
 */
void pwm_timer_deinit(PWM_Timer_t *pwm);

/* ==========================================================================
 * Channel Configuration
 * ========================================================================== */

/**
 * @brief Configure a PWM output channel
 * 
 * Sets up a timer channel for PWM output on the specified GPIO pin.
 * 
 * @param channel   Pointer to channel configuration structure
 * @param timer     Pointer to initialized timer
 * @param ch        Channel number (PWM_CHANNEL_1 to PWM_CHANNEL_4)
 * @param gpio_port GPIO port (e.g., GPIOB)
 * @param gpio_pin  GPIO pin (e.g., GPIO0)
 * @param mode      Output mode (push-pull or open-drain)
 * @return PWM_OK on success, error code otherwise
 * 
 * Example:
 * @code
 *     PWM_Channel_Config_t motor_a;
 *     pwm_channel_init(&motor_a, &motor_pwm, PWM_CHANNEL_3, GPIOB, GPIO0, PWM_OUTPUT_OPEN_DRAIN);
 * @endcode
 */
PWM_Status_t pwm_channel_init(PWM_Channel_Config_t *channel, PWM_Timer_t *timer,
                              PWM_Channel_t ch, uint32_t gpio_port, uint16_t gpio_pin,
                              PWM_OutputMode_t mode);

/**
 * @brief Enable PWM channel output
 * 
 * @param channel   Pointer to channel configuration
 */
void pwm_channel_enable(PWM_Channel_Config_t *channel);

/**
 * @brief Disable PWM channel output
 * 
 * @param channel   Pointer to channel configuration
 */
void pwm_channel_disable(PWM_Channel_Config_t *channel);

/* ==========================================================================
 * Duty Cycle Control
 * ========================================================================== */

/**
 * @brief Set PWM duty cycle (raw value)
 * 
 * Sets the duty cycle using the timer's native resolution.
 * 
 * @param channel   Pointer to channel configuration
 * @param duty      Duty cycle value (0 to timer->resolution)
 * 
 * @note For a timer with resolution=1000, duty=500 gives 50% duty cycle.
 */
void pwm_set_duty(PWM_Channel_Config_t *channel, uint16_t duty);

/**
 * @brief Set PWM duty cycle (percentage)
 * 
 * Sets the duty cycle as a percentage with 0.1% resolution.
 * 
 * @param channel   Pointer to channel configuration
 * @param percent   Duty cycle in tenths of percent (0-1000, where 1000 = 100.0%)
 * 
 * Example:
 * @code
 *     pwm_set_duty_percent(&motor_a, 500);  // 50.0% duty cycle
 *     pwm_set_duty_percent(&motor_a, 255);  // 25.5% duty cycle
 * @endcode
 */
void pwm_set_duty_percent(PWM_Channel_Config_t *channel, uint16_t percent);

/**
 * @brief Set PWM duty cycle (8-bit value)
 * 
 * Convenience function for 8-bit duty cycle control (0-255 range).
 * 
 * @param channel   Pointer to channel configuration
 * @param duty8     Duty cycle (0 = 0%, 255 = 100%)
 */
void pwm_set_duty_u8(PWM_Channel_Config_t *channel, uint8_t duty8);

/**
 * @brief Get current duty cycle
 * 
 * @param channel   Pointer to channel configuration
 * @return Current duty cycle value (0 to timer->resolution)
 */
uint16_t pwm_get_duty(const PWM_Channel_Config_t *channel);

/* ==========================================================================
 * Timer Control
 * ========================================================================== */

/**
 * @brief Start PWM timer
 * 
 * @param pwm   Pointer to timer configuration
 */
void pwm_timer_start(PWM_Timer_t *pwm);

/**
 * @brief Stop PWM timer
 * 
 * @param pwm   Pointer to timer configuration
 */
void pwm_timer_stop(PWM_Timer_t *pwm);

/**
 * @brief Update PWM frequency
 * 
 * Changes the PWM frequency while preserving duty cycle percentages.
 * 
 * @param pwm           Pointer to timer configuration
 * @param frequency_hz  New frequency in Hz
 * @return PWM_OK on success, error code otherwise
 */
PWM_Status_t pwm_set_frequency(PWM_Timer_t *pwm, uint32_t frequency_hz);

/**
 * @brief Get actual PWM frequency
 * 
 * @param pwm   Pointer to timer configuration
 * @return Actual PWM frequency in Hz
 */
uint32_t pwm_get_frequency(const PWM_Timer_t *pwm);

#ifdef __cplusplus
}
#endif

#endif /* PWM_H */
