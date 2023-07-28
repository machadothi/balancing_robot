#ifndef MPU6050_IMU_H_
#define MPU6050_IMU_H_

#include "imu.h"

void mpu6050_init();
float mpu6050_pitch();
float mpu6050_roll();
float mpu6050_yaw();

IMU *get_mpu6050_imu();

#endif // MPU6050_IMU_H_