// imu.h
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

typedef enum {
    IMU_Ok = 0,
    IMU_Config_Error,
    IMU_COMM_BUS_ERROR,
    IMU_Read_Timeout,
    IMU_Busy_Timeout
} IMU_Fails_t;

typedef struct {
    IMU_Fails_t (*init)(void);
    uint8_t (*id)(void);
    int16_t(*acc_x)(void);
    int16_t(*acc_y)(void);
    int16_t(*acc_z)(void);
    int16_t(*gyro_x)(void);
    int16_t(*gyro_y)(void);
    int16_t(*gyro_z)(void);
} IMU_t;

IMU_Fails_t imu_init(IMU_t *imu);
uint8_t imu_id(IMU_t *imu);
int16_t imu_acc_x(IMU_t *imu);
int16_t imu_acc_y(IMU_t *imu);
int16_t imu_acc_z(IMU_t *imu);

int16_t imu_gyro_x(IMU_t *imu);
int16_t imu_gyro_y(IMU_t *imu);
int16_t imu_gyro_z(IMU_t *imu);

#endif // IMU_H_