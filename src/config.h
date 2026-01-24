/**
 * @file config.h
 * @brief System-wide configuration and constants
 * 
 * This file contains all configurable parameters for the balancing robot.
 * Modify these values to tune the system for your specific hardware setup.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Hardware Configuration
 * ========================================================================== */

/** @defgroup Clock_Config Clock Configuration
 *  @brief System clock settings
 *  @{
 */
#define SYSTEM_CLOCK_MHZ        72      /**< System clock frequency in MHz */
/** @} */

/** @defgroup UART_Config UART Configuration
 *  @brief UART communication settings
 *  @{
 */
#define UART_BAUDRATE           921600  /**< UART baud rate for debug output */
#define UART_TX_QUEUE_SIZE      256     /**< UART transmit queue size (bytes) */
/** @} */

/** @defgroup I2C_Config I2C Configuration
 *  @brief I2C bus settings
 *  @{
 */
#define I2C_SPEED_KHZ           100     /**< I2C bus speed in kHz */
#define I2C_TIMEOUT_MS          100     /**< I2C operation timeout in ms */
/** @} */

/* ==========================================================================
 * IMU Configuration
 * ========================================================================== */

/** @defgroup IMU_Config IMU Configuration
 *  @brief IMU sampling and calibration settings
 *  @{
 */
#define IMU_SAMPLE_RATE_MS      10      /**< IMU sample period in ms (100Hz) */
#define IMU_SAMPLE_RATE_S       (IMU_SAMPLE_RATE_MS / 1000.0f)
#define IMU_QUEUE_SIZE          16      /**< IMU data queue size (samples) */

/** Gyroscope calibration offset (degrees/second)
 *  Measure with IMU stationary and adjust to get ~0 output */
#define GYRO_CALIBRATION_OFFSET -0.69f
/** @} */

/* ==========================================================================
 * Filter Configuration
 * ========================================================================== */

/** @defgroup Kalman_Config Kalman Filter Configuration
 *  @brief Kalman filter tuning parameters
 *  @{
 */
#define KALMAN_Q_ANGLE          0.1f    /**< Process noise (trust in gyro) */
#define KALMAN_R_MEASURE        0.5f    /**< Measurement noise (trust in accel) */
/** @} */

/** @defgroup Comp_Config Complementary Filter Configuration
 *  @brief Complementary filter tuning parameters
 *  @{
 */
#define COMPLEMENTARY_ALPHA     0.96f   /**< Filter coefficient (0.90-0.99) */
/** @} */

/* ==========================================================================
 * Motor Configuration
 * ========================================================================== */

/** @defgroup Motor_Config Motor Configuration
 *  @brief Motor driver settings
 *  @{
 */
#define MOTOR_PWM_FREQUENCY     500     /**< PWM frequency in Hz */
#define MOTOR_MAX_SPEED         255     /**< Maximum motor speed (0-255) */
/** @} */

/* ==========================================================================
 * FreeRTOS Task Configuration
 * ========================================================================== */

/** @defgroup Task_Config FreeRTOS Task Configuration
 *  @brief Task stack sizes and priorities
 *  @{
 */

/* Stack sizes (in words, not bytes) */
#define TASK_STACK_LED          128     /**< LED task stack size */
#define TASK_STACK_UART         256     /**< UART task stack size */
#define TASK_STACK_IMU          400     /**< IMU task stack size */
#define TASK_STACK_ROBOT        512     /**< Robot control task stack size */
#define TASK_STACK_MOTOR        256     /**< Motor demo task stack size */

/* Task names (for debugging) */
#define TASK_NAME_LED           "LED"
#define TASK_NAME_UART          "UART"
#define TASK_NAME_IMU           "IMU"
#define TASK_NAME_ROBOT         "ROBOT"
#define TASK_NAME_MOTOR         "MOTOR"
/** @} */

/* ==========================================================================
 * Debug Configuration
 * ========================================================================== */

/** @defgroup Debug_Config Debug Configuration
 *  @brief Debug and logging settings
 *  @{
 */
#define DEBUG_BUFFER_SIZE       150     /**< Debug output buffer size */
#define LOG_BUFFER_SIZE         256     /**< Log message buffer size */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
