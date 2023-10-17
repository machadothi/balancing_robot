// imu.h
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef struct {
    void (*init)(void);
    float (*pitch)(void);
    float (*roll)(void);
    float (*yaw)(void);
    uint8_t (*id)(void);
    uint16_t(*acc_x)(void);
} IMU;

void imu_init(IMU *imu);
float imu_pitch(IMU *imu);
float imu_roll(IMU *imu);
float imu_yaw(IMU *imu);
uint8_t imu_id(IMU *imu);
uint16_t imu_acc_x(IMU *imu);

#endif // IMU_H_