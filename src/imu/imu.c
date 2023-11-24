#include <math.h>

#include <FreeRTOS.h>
#include <task.h>

#include "communication/uart.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "imu.h"

#define MPU_RESOLUTION_SCALE 16384.0 // TODO: MOVE TO MPU IMPLEMENTATION
#define RAD_TO_DEGREE 180.0 / 3.14

IMU_Fails_t imu_init(IMU_t *imu) {
    // Call the init function pointed by the passed IMU_t structure    
    return imu->init();
}

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Data_t;

typedef struct {
    float x;
    float y;
} Angle_t;

static IMU_t *imu;
static IMU_Data_t imu_data;
static Angle_t angles;



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

static void calc_angle() {
    // Since the rotation around X-axes is always zero due to construction reason
    // the equation simplifies to:
    angles.y = atan(imu_data.acc_y/fabs(imu_data.acc_z)) * RAD_TO_DEGREE;
}

// -----------------------------------------------------------------------------

static void read_imu(IMU_t *imu) {
    imu_data.acc_x = imu_acc_x(imu) / MPU_RESOLUTION_SCALE;
    imu_data.acc_y = imu_acc_y(imu) / MPU_RESOLUTION_SCALE;
    imu_data.acc_z = imu_acc_z(imu) / MPU_RESOLUTION_SCALE;
    imu_data.gyro_x = imu_gyro_x(imu) / MPU_RESOLUTION_SCALE;
    imu_data.gyro_y = imu_gyro_y(imu) / MPU_RESOLUTION_SCALE;
    imu_data.gyro_z = imu_gyro_z(imu) / MPU_RESOLUTION_SCALE;
}

// -----------------------------------------------------------------------------

/*********************************************************************
 * Demo Task:
 *    Simply queues up two line messages to be TX, one second
 *    apart.
 *********************************************************************/
void
imu_demo_task(void *args __attribute__((unused))) {

    log_message(DEBUG, UART_BUS, "Starting IMU demo task");

    imu = get_mpu6050_imu();

    // TODO: Log error message
    IMU_Fails_t status = imu_init(imu);

    while(status) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uart_puts("Fail to init IMU\n\n\r");
    }


    char buffer[100];

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        uint8_t id = imu_id(imu);

        read_imu(imu);
        calc_angle();

        sprintf(buffer, "id: 0x%x | deg: %.2f | ax: %.2f | ay: %.2f | az: %.2f | gx: %.2f"
          " | gy: %.2f | gz: %.2f\n\r", id, angles.y, imu_data.acc_x, imu_data.acc_y, \
          imu_data.acc_z, imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
        uart_puts(buffer);

        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(100));
    }
}