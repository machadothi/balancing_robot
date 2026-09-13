/**
 * @file config.h
 * @brief System-wide configuration and constants
 *
 * Fixed constants live here. Build options (board, feature flags, baud rate,
 * sample rates) are CMake options generated into app_config.h - see
 * docs/02-build-and-configuration.md.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Hardware Configuration
 * ========================================================================== */

/** @defgroup UART_Config UART Configuration
 *  @brief UART communication settings
 *  @{
 */
#if defined(STM32F1)
#define UART_USB_TX_BUFFER_SIZE 1024    /**< USB console TX ring buffer (bytes, power of two) */
#define TELEMETRY_QUEUE_SIZE    16      /**< Records buffered for the telemetry task */
#else
#define UART_USB_TX_BUFFER_SIZE 4096
#define TELEMETRY_QUEUE_SIZE    32
#endif // defined(STM32F1)
#define UART_BT_TX_BUFFER_SIZE  512     /**< Bluetooth console TX ring buffer (boards with BOARD_BT_UART) */
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
#define IMU_SAMPLE_RATE_S       (IMU_SAMPLE_RATE_MS / 1000.0f)
#define IMU_QUEUE_SIZE          1       /**< Latest-sample mailbox: control never works through a backlog */
#define IMU_STALL_TIMEOUT_MS    (5 * IMU_SAMPLE_RATE_MS)  /**< No sample for this long: motors are stopped */
#define IMU_DLPF_MODE           MPU6050_DLPF_BW_42        /**< ~42 Hz sensor bandwidth, ~4.8 ms delay */

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
#define TASK_STACK_LED          64      /**< LED task stack size */
#define TASK_STACK_TELEMETRY    256     /**< Telemetry task stack size (line formatting) */
#define TASK_STACK_UART_RX      384     /**< UART RX task stack size (AT cmd + float printf) */
#define TASK_STACK_IMU          192     /**< IMU task stack size */
#define TASK_STACK_ROBOT        256     /**< Robot control task stack size (filters, PID) */

/* Task names (for debugging) */
#define TASK_NAME_LED           "LED"
#define TASK_NAME_TELEMETRY     "TELEM"
#define TASK_NAME_UART_RX       "UART_RX"
#define TASK_NAME_IMU           "IMU"
#define TASK_NAME_ROBOT         "ROBOT"

/* Priorities (configMAX_PRIORITIES = 5): the sensing/control chain must never
 * wait behind console I/O, and the heartbeat runs only when nothing else does */
#define TASK_PRIORITY_CONTROL   4       /**< IMU and robot control tasks */
#define TASK_PRIORITY_IO        2       /**< UART RX (AT commands) and telemetry tasks */
#define TASK_PRIORITY_LED       1       /**< Heartbeat LED */
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
#define WATCHDOG_TIMEOUT_MS     500     /**< Independent watchdog timeout (WATCHDOG_ENABLED) */
/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CONFIG_H
