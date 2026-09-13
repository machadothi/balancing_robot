/**
 * @file imu.h
 * @brief IMU service: sensor-agnostic sampling task and latest-sample mailbox
 *
 * A sensor driver exports an IMU_Ops_t that returns one sample in physical
 * units. The IMU task reads it at IMU_SAMPLE_RATE_MS and publishes the newest
 * sample; consumers wait for it with imu_wait_sample() and never see the queue.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>

#include "config.h"

/* ==========================================================================
 * Configuration (from config.h)
 * ========================================================================== */

#ifndef IMU_SAMPLE_RATE_MS
#define IMU_SAMPLE_RATE_MS  10
#endif // !IMU_SAMPLE_RATE_MS

#ifndef IMU_SAMPLE_RATE_S
#define IMU_SAMPLE_RATE_S   (IMU_SAMPLE_RATE_MS / 1000.0f)
#endif // !IMU_SAMPLE_RATE_S

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief IMU operation status codes
 */
typedef enum {
    IMU_OK = 0,             /**< Operation successful */
    IMU_CONFIG_ERROR,       /**< Configuration failed */
    IMU_COMM_BUS_ERROR,     /**< Communication bus error */
    IMU_READ_TIMEOUT,       /**< Read operation timed out */
    IMU_BUSY_TIMEOUT        /**< Device busy timeout */
} IMU_Status_t;

/**
 * @brief One sample in physical units
 *
 * - Accelerometer: g (1 g = 9.81 m/s²)
 * - Gyroscope: degrees/second
 */
typedef struct {
    float acc_x;    /**< X acceleration (g) */
    float acc_y;    /**< Y acceleration (g) */
    float acc_z;    /**< Z acceleration (g) */
    float gyro_x;   /**< X angular rate (°/s) */
    float gyro_y;   /**< Y angular rate (°/s) */
    float gyro_z;   /**< Z angular rate (°/s) */
} IMU_Data_t;

/**
 * @brief What a sensor driver provides
 *
 * Adding a sensor means implementing these two functions; nothing above the
 * IMU service changes.
 */
typedef struct {
    const char *name;                       /**< Sensor name, for diagnostics */
    IMU_Status_t (*init)(void);             /**< Bring up the bus and configure the sensor */
    IMU_Status_t (*read)(IMU_Data_t *out);  /**< One sample, scaled, uncalibrated */
} IMU_Ops_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/** Create the sample mailbox; call before the scheduler starts */
void imu_queue_init(void);

/**
 * @brief Wait for the newest sample
 * @param out      Filled with the sample on success
 * @param timeout  Longest wait, in ticks
 * @return false if no sample arrived within the timeout
 */
bool imu_wait_sample(IMU_Data_t *out, TickType_t timeout);

/**
 * @brief IMU acquisition task
 *
 * Initializes the sensor (retrying every second), then reads it every
 * IMU_SAMPLE_RATE_MS, applies the gyro calibration and publishes the sample.
 * A failed read publishes nothing, so consumers time out instead of acting
 * on a repeated old sample.
 */
void imu_task(void *args);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // IMU_H
