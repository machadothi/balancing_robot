#include <stdio.h>

#include "robot.h"
#include "imu/mpu6050.h"
#include "communication/i2c.h"

int main() {

    IMU *imu = get_mpu6050_imu();
    CommBus *comm = get_i2c_bus();

    imu_init(imu);
    comm_bus_init(comm);

    float pitch = imu_pitch(imu);
    float roll = imu_roll(imu);
    float yaw = imu_yaw(imu);

    printf("imu values: %f %f %f\n", pitch, roll, yaw);

    comm_bus_write_byte(comm, 0x00);
    uint8_t a = comm_bus_read_byte(comm);
    comm_bus_start(comm);
    comm_bus_stop(comm);

    return 0;
}