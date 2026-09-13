/**
 * @file mpu6050.h
 * @brief MPU6050 6-axis IMU driver
 * 
 * Driver for the InvenSense MPU6050 3-axis accelerometer and 3-axis gyroscope.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "imu/imu.h"
#include "imu/mpu6050_regs.h"

/* ==========================================================================
 * Sensor Configuration
 * ========================================================================== */

/** Accelerometer sensitivity for ±2g range (LSB/g) */
#define ACC_SENS_SCALE_FACTOR   16384.0f

/** Gyroscope sensitivity for ±250°/s range (LSB/(°/s)) */
#define GYRO_SENS_SCALE_FACTOR  131.0f

/* Gyroscope calibration offset: GYRO_CALIBRATION_OFFSET in config.h */

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Get MPU6050 driver interface
 * @return Pointer to IMU driver structure
 */
IMU_Driver_t *mpu6050_get_driver(void);

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */

IMU_Status_t mpu6050_init(void);

/**
 * @brief Set the up reset pin object. It was added a ON/OFF pin to the VCC pin
 * of the MPU6050 to make a hard reset.
 */
void mpu6050_setup_reset_pin(void);

/**
 * @brief Set the reset bit.
 */
void mpu6050_hard_reset(void);

/**
 * @brief Set the reset bit.
 */
void mpu6050_soft_reset(void);

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */

bool mpu6050_test_connection(void);

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t mpu6050_get_device_id(void);

/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
void mpu6050_set_device_id(uint8_t id);

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
IMU_Status_t mpu6050_set_clock_source(uint8_t source);

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
IMU_Status_t mpu6050_set_full_scale_gyro_range(uint8_t range);

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
IMU_Status_t mpu6050_set_full_scale_accel_range(uint8_t range);

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
IMU_Status_t mpu6050_set_sleep_enabled(bool enabled);

/**
 * @brief Set the digital low-pass filter (CONFIG register, DLPF_CFG)
 * @param mode  One of the MPU6050_DLPF_BW_* bandwidths
 */
IMU_Status_t mpu6050_set_dlpf_mode(uint8_t mode);

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
int16_t mpu6050_get_acceleration_x(void);

/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_YOUT_H
 */
int16_t mpu6050_get_acceleration_y(void);

/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_ZOUT_H
 */
int16_t mpu6050_get_acceleration_z(void);

/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
int16_t mpu6050_get_rotation_x(void);

/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
int16_t mpu6050_get_rotation_y(void);

/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
int16_t mpu6050_get_rotation_z(void);

/**
 * @brief Read all sensor data via DMA (14 bytes burst)
 * 
 * Performs a single DMA transfer to read all accelerometer and gyroscope data.
 * This is more efficient than individual axis reads.
 * After this call, mpu6050_get_acceleration_x/y/z and mpu6050_get_rotation_x/y/z return cached values.
 * 
 * @return IMU_Status_t IMU_OK on success, error code otherwise
 */
IMU_Status_t mpu6050_read_all_dma(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // MPU6050_H