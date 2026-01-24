/**
 * @file robot.c
 * @brief Robot control task implementation
 * 
 * Implements the main control loop for the self-balancing robot.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"
#include "robot/robot.h"
#include "imu/imu.h"
#include "filter/filter.h"
#include "communication/uart.h"
#include "log/log.h"

/* ==========================================================================
 * Constants
 * ========================================================================== */

#define RAD_TO_DEG  (180.0f / 3.14159265f)

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate tilt angle from accelerometer data
 * 
 * Uses atan2 for proper quadrant handling.
 * IMU is mounted with X-axis vertical (ax ≈ -1g when level).
 * 
 * @param data Pointer to IMU data structure
 * @return Tilt angle in degrees
 */
static float calc_angle_from_accel(const IMU_Data_t *data) {
    return atan2f(data->acc_y, -data->acc_x) * RAD_TO_DEG;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void robot_task(void *args) {
    (void)args;
    
    IMU_Data_t imu_data;
    char buffer[DEBUG_BUFFER_SIZE];
    
    /* Initialize filters */
    KalmanFilter_t kalman;
    ComplementaryFilter_t complementary;
    
    kalman_init(&kalman);
    complementary_init(&complementary);

    for (;;) {
        if (xQueueReceive(imu_content, &imu_data, 500) == pdPASS) {
            /* Calculate angle from accelerometer */
            float acc_angle = calc_angle_from_accel(&imu_data);
            
            /* Update both filters */
            float kalman_angle = kalman_update(&kalman, imu_data.gyro_x, 
                acc_angle, IMU_SAMPLE_RATE_S);
            float comp_angle = complementary_update(&complementary, imu_data.gyro_x, 
                acc_angle, IMU_SAMPLE_RATE_S);

            /* Output debug data */
            snprintf(buffer, sizeof(buffer), 
                "ax: %.2f | ay: %.2f | az: %.2f | gx: %.2f"
                " | gy: %.2f | gz: %.2f | acc_deg: %.2f | kalman: %.2f | comp: %.2f",
                imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, 
                imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, 
                acc_angle, kalman_angle, comp_angle);

            log_message(LOG_DEBUG, ROBOT_TASK, buffer);
        } else {
            taskYIELD();
        }
    }
}