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
#include "app/module.h"
#include "imu/imu.h"
#if IMU_SENSOR_QMI8658
#include "imu/qmi8658.h"
#else
#include "imu/mpu6050.h"
#endif // IMU_SENSOR_QMI8658
#include "log/log.h"

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** The sensor in use: the only line that names a specific driver */
#if IMU_SENSOR_QMI8658
static const IMU_Ops_t *const sensor = &qmi8658_ops;
#else
static const IMU_Ops_t *const sensor = &mpu6050_ops;
#endif // IMU_SENSOR_QMI8658

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

    IMU_Status_t status;
    while ((status = sensor->init()) != IMU_OK) {
        /* IMU_Status_t: 1 config (no ACK), 2 bus (lines stuck), 3/4 timeouts */
        log_message_with_int(LOG_ERROR, IMU_TASK, "IMU init failed, retrying; status", (int)status);
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

APP_MODULE(imu_module) = {
    .name = "IMU",
    .init = imu_queue_init,
    .task = imu_task,
    .stack = 192,
    .priority = APP_PRIORITY_CONTROL,
};
