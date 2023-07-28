#include <stdio.h>

#include "robot.h"
#include "imu/mpu6050.h"

int main() {

    IMU *imu = get_mpu6050_imu();

    imu_init(imu);

    float pitch = imu_pitch(imu);
    float roll = imu_roll(imu);
    float yaw = imu_yaw(imu);

    printf("imu values: %f %f %f\n", pitch, roll, yaw);

    return 0;
}