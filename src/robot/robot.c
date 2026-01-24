#include <stdio.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "robot.h"
#include "imu/imu.h"
#include "communication/uart.h"
#include "log/log.h"

// -----------------------------------------------------------------------------

#define RAD_TO_DEGREE (180.0f / 3.14159f)

// -----------------------------------------------------------------------------

static float calc_angle(IMU_Data_t *imu_data_) {
    // IMU is mounted with X-axis vertical (ax ≈ -1g when level)
    // Use atan2 for proper quadrant handling and avoid division by zero
    return atan2f(imu_data_->acc_y, -imu_data_->acc_x) * RAD_TO_DEGREE;
}

// -----------------------------------------------------------------------------

static void kalman_filter(float *k_state, float *k_uncert, float gyro_rate, float acc_angle) {
    
    // Process noise - how much we trust the gyro integration
    // Higher value = more responsive to changes, but noisier
    const float Q_angle = 0.1f;  // Process noise variance for angle
    
    // Measurement noise - how much we trust the accelerometer
    // Higher value = trust accelerometer less, smoother output  
    const float R_measure = 0.5f;  // Measurement noise variance
    
    // Predict step: integrate gyro rate to get angle
    *k_state = *k_state + (gyro_rate * SAMPLE_RATE_S);
    
    // Update uncertainty (increases with time)
    *k_uncert = *k_uncert + Q_angle;
    
    // Update step: correct with accelerometer measurement
    // Calculate Kalman gain
    float K = *k_uncert / (*k_uncert + R_measure);
    
    // Update estimate with measurement
    *k_state = *k_state + K * (acc_angle - *k_state);
    
    // Update uncertainty (decreases after measurement)
    *k_uncert = (1.0f - K) * *k_uncert;
}

// -----------------------------------------------------------------------------

void robot_task(void *args __attribute__((unused))) {

    IMU_Data_t d;
    static float acc_y_degree = 0;
    static float gyro_x_degree = 0;
    char buffer[100];
    float kalman_angle = 0;
    float kalman_uncertainty = 1.0f;  // Start with high uncertainty

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