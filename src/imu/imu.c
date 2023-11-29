#include <math.h>

#include <FreeRTOS.h>
#include <task.h>

#include "communication/uart.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "imu.h"

// -----------------------------------------------------------------------------

QueueHandle_t imu_content;

// -----------------------------------------------------------------------------

typedef struct {
    float x;
    float y;
} Angle_t;

static IMU_t *imu_;

// -----------------------------------------------------------------------------

IMU_Fails_t imu_init(IMU_t *imu) {
    imu_content = xQueueCreate(256,sizeof(IMU_Data_t));
    return imu->init();
}

// -----------------------------------------------------------------------------

uint8_t imu_id(IMU_t *imu) {
    return imu->id();
}

// -----------------------------------------------------------------------------

int16_t imu_acc_x(IMU_t *imu) {
    return imu->acc_x();
}

// -----------------------------------------------------------------------------

int16_t imu_acc_y(IMU_t *imu) {
    return imu->acc_y();
}

// -----------------------------------------------------------------------------

int16_t imu_acc_z(IMU_t *imu) {
    return imu->acc_z();
}

// -----------------------------------------------------------------------------

int16_t imu_gyro_x(IMU_t *imu) {
    return imu->gyro_x();
}

// -----------------------------------------------------------------------------

int16_t imu_gyro_y(IMU_t *imu) {
    return imu->gyro_y();
}

// -----------------------------------------------------------------------------

int16_t imu_gyro_z(IMU_t *imu) {
    return imu->gyro_z();
}

// -----------------------------------------------------------------------------

static void read_imu(IMU_t *imu, IMU_Data_t *imu_data_) {
    imu_data_->acc_x = imu_acc_x(imu) / ACC_SENS_SCALE_FACTOR;
    imu_data_->acc_y = imu_acc_y(imu) / ACC_SENS_SCALE_FACTOR;
    imu_data_->acc_z = imu_acc_z(imu) / ACC_SENS_SCALE_FACTOR;
    imu_data_->gyro_x = (imu_gyro_x(imu) / GYRO_SENS_SCALE_FACTOR) + GYRO_CONST_ERROR_MEAS;
    imu_data_->gyro_y = imu_gyro_y(imu) / GYRO_SENS_SCALE_FACTOR;
    imu_data_->gyro_z = imu_gyro_z(imu) / GYRO_SENS_SCALE_FACTOR;
}

// -----------------------------------------------------------------------------

static void
send_imu_data(const IMU_Data_t *data) {
    xQueueSend(imu_content, data, portMAX_DELAY);
}

// -----------------------------------------------------------------------------

/*********************************************************************
 * Reads data from the IMU and populate its QUEUE
 *********************************************************************/
void
imu_task(void *args __attribute__((unused))) {

    log_message(DEBUG, UART_BUS, "Starting IMU demo task");

    imu_ = get_mpu6050_imu();
    
    static IMU_Data_t imu_data;
    static Angle_t imu_angles;

    // TODO: Log error message
    IMU_Fails_t status = imu_init(imu_);

    while(status) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uart_puts("Fail to init IMU\n\n\r");
    }

    char buffer[100];

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        uint8_t id = imu_id(imu_);

        read_imu(imu_, &imu_data);

        send_imu_data(&imu_data);

        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(100));
    }
}