/**
 * @file imu.h
 * @brief Generic IMU (Inertial Measurement Unit) interface
 * 
 * Provides a hardware-agnostic interface for IMU sensors.
 * Currently supports MPU6050, but designed to be extensible.
 * 
 * The IMU task reads sensor data at a fixed rate and sends it
 * to a FreeRTOS queue for processing by other tasks.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

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
 * @brief IMU driver interface (function pointers)
 * 
 * Allows different IMU implementations to be used interchangeably.
 */
typedef struct {
    IMU_Status_t (*init)(void);     /**< Initialize the IMU */
    uint8_t (*id)(void);            /**< Get device ID */
    IMU_Status_t (*read_all)(void); /**< Read all sensors via DMA (bulk read) */
    int16_t (*acc_x)(void);         /**< Read X accelerometer (raw) */
    int16_t (*acc_y)(void);         /**< Read Y accelerometer (raw) */
    int16_t (*acc_z)(void);         /**< Read Z accelerometer (raw) */
    int16_t (*gyro_x)(void);        /**< Read X gyroscope (raw) */
    int16_t (*gyro_y)(void);        /**< Read Y gyroscope (raw) */
    int16_t (*gyro_z)(void);        /**< Read Z gyroscope (raw) */
} IMU_Driver_t;

/**
 * @brief Processed IMU data structure
 * 
 * Contains calibrated sensor values in physical units:
 * - Accelerometer: g (1g = 9.81 m/s²)
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

/* ==========================================================================
 * Public Variables
 * ========================================================================== */

/** Queue for passing IMU data to consumer tasks */
extern QueueHandle_t imu_content;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize IMU data queue
 * 
 * Must be called before starting the IMU task.
 * Creates a FreeRTOS queue for IMU data.
 */
void imu_queue_init(void);

/**
 * @brief Initialize IMU hardware
 * @param imu Pointer to IMU driver interface
 * @return IMU_OK on success, error code otherwise
 */
IMU_Status_t imu_init(IMU_Driver_t *imu);

/**
 * @brief Get IMU device ID
 * @param imu Pointer to IMU driver interface
 * @return Device ID byte
 */
uint8_t imu_id(IMU_Driver_t *imu);

/**
 * @brief Read raw X accelerometer value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit accelerometer value
 */
int16_t imu_acc_x(IMU_Driver_t *imu);

/**
 * @brief Read raw Y accelerometer value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit accelerometer value
 */
int16_t imu_acc_y(IMU_Driver_t *imu);

/**
 * @brief Read raw Z accelerometer value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit accelerometer value
 */
int16_t imu_acc_z(IMU_Driver_t *imu);

/**
 * @brief Read raw X gyroscope value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit gyroscope value
 */
int16_t imu_gyro_x(IMU_Driver_t *imu);

/**
 * @brief Read raw Y gyroscope value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit gyroscope value
 */
int16_t imu_gyro_y(IMU_Driver_t *imu);

/**
 * @brief Read raw Z gyroscope value
 * @param imu Pointer to IMU driver interface
 * @return Raw 16-bit gyroscope value
 */
int16_t imu_gyro_z(IMU_Driver_t *imu);

/**
 * @brief IMU data acquisition task
 * 
 * FreeRTOS task that:
 * 1. Initializes the IMU hardware
 * 2. Reads sensor data at IMU_SAMPLE_RATE_MS intervals
 * 3. Converts raw values to physical units
 * 4. Sends data to the imu_content queue
 * 
 * @param args Task arguments (unused)
 */
void imu_task(void *args);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // IMU_H
