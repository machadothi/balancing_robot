#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "robot.h"
#include "imu/imu.h"
#include "log/log.h"

#define RAD_TO_DEGREE 180.0 / 3.14

static float calc_angle(IMU_Data_t *imu_data_) {
    // Since the rotation around X-axes is always zero due to construction reason
    // the equation simplifies to:
    return atan(imu_data_->acc_y/fabs(imu_data_->acc_z)) * RAD_TO_DEGREE;
}

void robot_task(void *args __attribute__((unused))) {

    IMU_Data_t d;
    static float acc_y_degree;
    char buffer[100];

    for (;;) {
        // Receive char to be TX
        if ( xQueueReceive(imu_content,&d,500) == pdPASS ) {

            acc_y_degree = calc_angle(&d);

            sprintf(buffer, "acc_deg: %.2f | ax: %.2f | ay: %.2f | az: %.2f | gx: %.2f"
                " | gy: %.2f | gz: %.2f\n\r", acc_y_degree, d.acc_x, d.acc_y, \
                d.acc_z, d.gyro_x, d.gyro_y, d.gyro_z);

            uart_puts(buffer);
        } else {
            taskYIELD();
        }
    }
}