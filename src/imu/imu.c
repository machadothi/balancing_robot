#include <FreeRTOS.h>
#include <task.h>

#include "communication/uart.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "imu.h"

IMU_Fails_t imu_init(IMU_t *imu) {
    // Call the init function pointed by the passed IMU_t structure    
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

/*********************************************************************
 * Demo Task:
 *    Simply queues up two line messages to be TX, one second
 *    apart.
 *********************************************************************/
void
imu_demo_task(void *args __attribute__((unused))) {
    LogDriver_t logDriver;
    logDriver.log_level = DEBUG;
    logDriver.send = uart_puts;
    
    log_init(&logDriver);
    log_message(DEBUG, UART_BUS, "Starting IMU demo task");

    IMU_t *imu = get_mpu6050_imu();

    // TODO: Log error message
    IMU_Fails_t status = imu_init(imu);

    while(status) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uart_puts("Fail to init IMU\n\n\r");
    }


    char buffer[7];  // Large enough for a 2-byte int and '\0'

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        uint8_t data = imu_id(imu);
        itoa(data, buffer, 16); // 10 for base 10 (decimal) representation

        uart_puts("  device id: 0x");
        uart_puts(buffer);
        uart_puts(".\n\n\r");

        int16_t acc_x = imu_acc_x(imu);
        itoa(acc_x, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_x: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t acc_y = imu_acc_y(imu);
        itoa(acc_y, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_y: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t acc_z = imu_acc_z(imu);
        itoa(acc_z, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_z: ");
        uart_puts(buffer);
        uart_puts(".\n\n\r");


        int16_t gyro_x = imu_gyro_x(imu);
        itoa(gyro_x, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_x: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t gyro_y = imu_gyro_y(imu);
        itoa(gyro_y, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_y: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t gyro_z = imu_gyro_z(imu);
        itoa(gyro_z, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_z: ");
        uart_puts(buffer);
        uart_puts(".\n\n\r");

        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(1000));
    }
}