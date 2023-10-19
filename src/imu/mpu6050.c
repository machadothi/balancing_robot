#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "communication/i2c.h"

#define NO_OPT __attribute__((optimize("O0")))

static I2C_Control i2c;            // I2C Control struct

IMU *get_mpu6050_imu(void) {
    static IMU mpu6050_imu = {
        .init = initialize,
        .id = getDeviceID,
        .acc_x = getAccelerationX,
        .acc_y = getAccelerationY,
        .acc_z = getAccelerationZ,
        .gyro_x = getRotationX,
        .gyro_y = getRotationY,
        .gyro_z = getRotationZ
    };
    return &mpu6050_imu;
}

// -----------------------------------------------------------------------------

IMU_Fails NO_OPT
initialize(void) {
    i2c_setup_peripheral();
    setup_reset_pin();
    hardReset();
    i2c_configure(&i2c, I2C1, MPU6050_DEFAULT_ADDRESS, 100);

    if(setClockSource(MPU6050_CLOCK_PLL_XGYRO)) {
        return IMU_COMM_BUS_ERROR;
    }
    if(setFullScaleGyroRange(MPU6050_GYRO_FS_250)){
        return IMU_COMM_BUS_ERROR;
    }
    if(setFullScaleAccelRange(MPU6050_ACCEL_FS_2)){
        return IMU_COMM_BUS_ERROR;
    }
    if(setSleepEnabled(false)){
        return IMU_COMM_BUS_ERROR;
    }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

void
setup_reset_pin(void) {
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Set GPIO10 (in GPIO port A) to 'output push-pull'. */
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL,GPIO10);
}

// -----------------------------------------------------------------------------

void
hardReset(void) {
    gpio_clear(GPIOA,GPIO10);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set(GPIOA,GPIO10); // enable ON
}

// -----------------------------------------------------------------------------

bool
testConnection(void) {
    return getDeviceID() == MPU6050_DEFAULT_ADDRESS;
}

// -----------------------------------------------------------------------------

uint8_t NO_OPT
getDeviceID(void) {
    uint8_t buffer = 0;
    i2c_read_byte(&i2c, MPU6050_RA_WHO_AM_I, &buffer);
    return buffer;
}

// -----------------------------------------------------------------------------

void setDeviceID(uint8_t id) {
    i2c_write_bits(&i2c, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// -----------------------------------------------------------------------------

void NO_OPT
softReset(void) {
    i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

// -----------------------------------------------------------------------------

IMU_Fails NO_OPT
setClockSource(uint8_t source) {

    if(i2c_write_bits(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, 
      MPU6050_PWR1_CLKSEL_LENGTH, source)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails NO_OPT
setFullScaleGyroRange(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, 
      MPU6050_GCONFIG_FS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails NO_OPT
setFullScaleAccelRange(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, 
      MPU6050_ACONFIG_AFS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails NO_OPT
setSleepEnabled(bool enabled) {
    if(i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 
      enabled)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

int16_t getAccelerationX(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_XOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t getAccelerationY(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_YOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t getAccelerationZ(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_ZOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t getRotationX(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_XOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t getRotationY(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_YOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t getRotationZ(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_ZOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}