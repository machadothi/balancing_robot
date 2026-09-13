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
 * Debug Configuration
 * ========================================================================== */

/** @defgroup Debug_Config Debug Configuration
 *  @brief Debug and logging settings
 *  @{
 */
#define LOG_BUFFER_SIZE         256     /**< Log message buffer size */
#define WATCHDOG_TIMEOUT_MS     500     /**< Independent watchdog timeout (WATCHDOG) */
#define AUTO_ENABLE_HOLD_MS     3000    /**< Upright time before AUTO_ENABLE starts balancing */
/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // CONFIG_H
