#include "imu.h"

void imu_init(IMU *imu) {
    // Call the init function pointed by the passed IMU structure    
    imu->init();
}

float imu_pitch(IMU *imu) {
    // Call the read_pitch function pointed by the passed IMU structure
    return imu->pitch();
}

float imu_roll(IMU *imu) {
    // Call the read_roll function pointed by the passed IMU structure
    return imu->roll();
}

float imu_yaw(IMU *imu) {
    // Call the read_yaw function pointed by the passed IMU structure
    return imu->yaw();
}

uint8_t imu_id(IMU *imu) {
    return imu->id();
}