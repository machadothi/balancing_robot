// imu.h
#ifndef IMU_H_
#define IMU_H_

typedef struct {
    void (*init)();
    float (*pitch)();
    float (*roll)();
    float (*yaw)();
} IMU;

void imu_init(IMU *imu);
float imu_pitch(IMU *imu);
float imu_roll(IMU *imu);
float imu_yaw(IMU *imu);

#endif // IMU_H_