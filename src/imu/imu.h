// imu.h
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef enum {
    IMU_Ok = 0,
    IMU_Configure,
    IMU_Read_Timeout,
    IMU_Busy_Timeout
} IMU_Fails;

typedef struct {
    IMU_Fails (*init)(void);
    uint8_t (*id)(void);
    int16_t(*acc_x)(void);
    int16_t(*acc_y)(void);
    int16_t(*acc_z)(void);
    int16_t(*gyro_x)(void);
    int16_t(*gyro_y)(void);
    int16_t(*gyro_z)(void);
} IMU;

IMU_Fails imu_init(IMU *imu);
uint8_t imu_id(IMU *imu);
int16_t imu_acc_x(IMU *imu);
int16_t imu_acc_y(IMU *imu);
int16_t imu_acc_z(IMU *imu);

int16_t imu_gyro_x(IMU *imu);
int16_t imu_gyro_y(IMU *imu);
int16_t imu_gyro_z(IMU *imu);

#endif // IMU_H_