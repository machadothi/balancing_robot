/**
 * @file imu.c
 * @brief IMU service: sampling task and latest-sample mailbox
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"
#include "imu/imu.h"
#include "imu/mpu6050.h"
#include "log/log.h"

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** The sensor in use: the only line that names a specific driver */
static const IMU_Ops_t *const sensor = &mpu6050_ops;

/** Holds one sample (IMU_QUEUE_SIZE 1): overwritten, never backlogged */
static QueueHandle_t samples = NULL;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void imu_queue_init(void) {
    samples = xQueueCreate(IMU_QUEUE_SIZE, sizeof(IMU_Data_t));
}

bool imu_wait_sample(IMU_Data_t *out, TickType_t timeout) {
    return xQueueReceive(samples, out, timeout) == pdPASS;
}

void imu_task(void *args) {
    (void)args;

    while (sensor->init() != IMU_OK) {
        log_message(LOG_ERROR, IMU_TASK, "IMU init failed, retrying");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    log_message(LOG_INFO, IMU_TASK, "IMU initialized");

    IMU_Data_t sample;

    /* Read once: vTaskDelayUntil then keeps a fixed period instead of period + work time */
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        if (sensor->read(&sample) == IMU_OK) {
            sample.gyro_x += GYRO_CALIBRATION_OFFSET;
            /* The controller always gets the latest measurement */
            xQueueOverwrite(samples, &sample);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_RATE_MS));
    }
}
