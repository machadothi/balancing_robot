#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "robot.h"
#include "imu/imu.h"
#include "log/log.h"

#define RAD_TO_DEGREE 180.0 / 3.14

static float
calc_angle(IMU_Data_t *imu_data_) {
    // Since the rotation around X-axes is always zero due to construction reason
    // the equation simplifies to:
    return atan(imu_data_->acc_y/fabs(imu_data_->acc_z)) * RAD_TO_DEGREE;
}

static void
kalman_filter(float *k_state, float *k_uncert, float gyro_x, float acc_angle) {

    // previosly calculated standard deviations
    float std_gyro = 0.1;
    float std_acc = 0.2;

    // predict the current state of the system
    *k_state = *k_state + (gyro_x * SAMPLE_RATE_S);

    // calculate the uncertainty of the prediction
    *k_uncert = *k_uncert + (pow(SAMPLE_RATE_S,2) * pow(std_gyro,2));

    // calculate Kalman's Gain
    float dummy = *k_uncert + (pow(SAMPLE_RATE_S,2) * pow(std_acc,2));
    float k = *k_uncert/dummy;

    // update the predicted state with Kalman's Gain
    *k_state = *k_state + k*(acc_angle - *k_state);

    // update uncertainty
    *k_uncert = (1 - k) * *k_uncert;
}

void
robot_task(void *args __attribute__((unused))) {

    IMU_Data_t d;
    static float acc_y_degree = 0;
    static float gyro_x_degree = 0;
    char buffer[100];
    float kalman_angle = 0;
    float kalman_uncertainty = 0;

    for (;;) {
        // Receive char to be TX
        if ( xQueueReceive(imu_content,&d,500) == pdPASS ) {

            acc_y_degree = calc_angle(&d);
            kalman_filter(&kalman_angle, &kalman_uncertainty, d.gyro_x, acc_y_degree);

            gyro_x_degree += d.gyro_x * 0.1;
            sprintf(buffer, "ax: %.2f | ay: %.2f | az: %.2f | gx: %.2f"
                " | gy: %.2f | gz: %.2f | acc_deg: %.2f | gyro_deg: %.2f\n\r",d.acc_x, d.acc_y, \
                d.acc_z, d.gyro_x, d.gyro_y, d.gyro_z, acc_y_degree, kalman_angle);

            uart_puts(buffer);
        } else {
            taskYIELD();
        }
    }
}