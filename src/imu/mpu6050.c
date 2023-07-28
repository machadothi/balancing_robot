#include "mpu6050.h"

void mpu6050_init() {
    // initialize I2C for specific IMU
}

float mpu6050_pitch() {
    // read and return pitch from specific IMU
    return 0.0;
}

float mpu6050_roll() {
    // read and return roll from specific IMU
    return 1.0;
}

float mpu6050_yaw() {
    // read and return yaw from specific IMU
    return 2.0;
}

IMU *get_mpu6050_imu() {
    static IMU mpu6050_imu = {
        .init = mpu6050_init,
        .pitch = mpu6050_pitch,
        .roll = mpu6050_roll,
        .yaw = mpu6050_yaw
    };
    return &mpu6050_imu;
}