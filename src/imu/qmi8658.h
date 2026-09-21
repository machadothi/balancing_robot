/**
 * @file qmi8658.h
 * @brief QST QMI8658 6-axis IMU driver (the IMU on the Hiwonder F407 board)
 *
 * The driver is used only through qmi8658_ops.
 */

#ifndef QMI8658_H
#define QMI8658_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "imu/imu.h"

/**
 * @brief QMI8658 on the board I2C bus
 *
 * init: I2C bus setup, WHO_AM_I check, soft reset, then ±2 g, ±256 °/s,
 *       224 Hz output rate with the internal low-pass filters at ~30 Hz.
 * read: one 12-byte DMA burst from AX_L, scaled to g and °/s.
 */
extern const IMU_Ops_t qmi8658_ops;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // QMI8658_H
