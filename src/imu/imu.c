/**
 * @file imu.c
 * @brief Generic IMU interface implementation
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <math.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "drivers/uart.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "imu/imu.h"

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

QueueHandle_t imu_content;

static IMU_Driver_t *imu_driver;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void imu_queue_init(void) {
    imu_content = xQueueCreate(IMU_QUEUE_SIZE, sizeof(IMU_Data_t));
}

IMU_Status_t imu_init(IMU_Driver_t *imu) {
    return imu->init();
}

uint8_t imu_id(IMU_Driver_t *imu) {
    return imu->id();
}

int16_t imu_acc_x(IMU_Driver_t *imu) {
    return imu->acc_x();
}

int16_t imu_acc_y(IMU_Driver_t *imu) {
    return imu->acc_y();
}

int16_t imu_acc_z(IMU_Driver_t *imu) {
    return imu->acc_z();
}

int16_t imu_gyro_x(IMU_Driver_t *imu) {
    return imu->gyro_x();
}

int16_t imu_gyro_y(IMU_Driver_t *imu) {
    return imu->gyro_y();
}

int16_t imu_gyro_z(IMU_Driver_t *imu) {
    return imu->gyro_z();
}

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Read and convert IMU data to physical units
 * 
 * Performs a single DMA burst read of all sensors, then extracts
 * the cached values. Much more efficient than 6 individual I2C reads.
 */
static bool read_imu_data(IMU_Driver_t *imu, IMU_Data_t *data) {
    /* Trigger DMA read of all sensor data at once; on failure the cached
     * values are from the previous sample and must not be reported as new */
    if (imu->read_all != NULL && imu->read_all() != IMU_OK) {
        return false;
    }
    
    /* Now read cached values (no I2C transactions) */
    data->acc_x = imu_acc_x(imu) / ACC_SENS_SCALE_FACTOR;
    data->acc_y = imu_acc_y(imu) / ACC_SENS_SCALE_FACTOR;
    data->acc_z = imu_acc_z(imu) / ACC_SENS_SCALE_FACTOR;
    data->gyro_x = (imu_gyro_x(imu) / GYRO_SENS_SCALE_FACTOR) + GYRO_CALIBRATION_OFFSET;
    data->gyro_y = imu_gyro_y(imu) / GYRO_SENS_SCALE_FACTOR;
    data->gyro_z = imu_gyro_z(imu) / GYRO_SENS_SCALE_FACTOR;
    return true;
}

/**
 * @brief Publish the newest sample
 *
 * The queue holds one element: overwriting means the controller always gets
 * the latest measurement instead of a backlog of old ones.
 */
static void send_imu_data(const IMU_Data_t *data) {
    xQueueOverwrite(imu_content, data);
}

/* ==========================================================================
 * Task Implementation
 * ========================================================================== */

void imu_task(void *args) {
    (void)args;
    
    log_message(LOG_DEBUG, IMU_TASK, "Starting IMU task");

    imu_driver = get_mpu6050_imu();
    
    IMU_Data_t imu_data;
    IMU_Status_t status = imu_init(imu_driver);

    /* Wait for successful initialization */
    while (status != IMU_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        log_message(LOG_ERROR, IMU_TASK, "Failed to init IMU, retrying...");
        status = imu_init(imu_driver);
    }
    
    log_message(LOG_INFO, IMU_TASK, "IMU initialized successfully");

    /* Read once: vTaskDelayUntil then keeps a fixed period instead of period + work time */
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        if (read_imu_data(imu_driver, &imu_data)) {
            send_imu_data(&imu_data);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_RATE_MS));
    }
}