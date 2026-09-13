/**
 * @file mpu6050.h
 * @brief MPU-6050 6-axis IMU driver
 *
 * The driver is used only through mpu6050_ops. Register definitions are in
 * mpu6050_regs.h.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "imu/imu.h"

/* ==========================================================================
 * Sensor Configuration
 * ========================================================================== */

/** Accelerometer sensitivity for ±2g range (LSB/g) */
#define ACC_SENS_SCALE_FACTOR   16384.0f

/** Gyroscope sensitivity for ±250°/s range (LSB/(°/s)) */
#define GYRO_SENS_SCALE_FACTOR  131.0f

/* Gyroscope calibration offset: GYRO_CALIBRATION_OFFSET in config.h */

/* ==========================================================================
 * Driver
 * ========================================================================== */

/**
 * @brief MPU-6050 on the board I2C bus
 *
 * init: I2C bus recovery and setup, then ±2 g, ±250 °/s, X-gyro clock,
 *       IMU_DLPF_MODE low-pass, sleep off.
 * read: one 14-byte DMA burst from ACCEL_XOUT_H, scaled to g and °/s.
 */
extern const IMU_Ops_t mpu6050_ops;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MPU6050_H
