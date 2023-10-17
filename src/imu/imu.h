// imu.h
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef struct {
    void (*init)(void);
    uint8_t (*id)(void);
    int16_t(*acc_x)(void);
    int16_t(*acc_y)(void);
    int16_t(*acc_z)(void);
    int16_t(*gyro_x)(void);
    int16_t(*gyro_y)(void);
    int16_t(*gyro_z)(void);
} IMU;

void imu_init(IMU *imu);
uint8_t imu_id(IMU *imu);
int16_t imu_acc_x(IMU *imu);
int16_t imu_acc_y(IMU *imu);
int16_t imu_acc_z(IMU *imu);

int16_t imu_gyro_x(IMU *imu);
int16_t imu_gyro_y(IMU *imu);
int16_t imu_gyro_z(IMU *imu);

#endif // IMU_H_