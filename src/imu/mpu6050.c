#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "communication/i2c.h"
#include "log/log.h"

#define NO_OPT __attribute__((optimize("O0")))

static I2C_Control_t i2c;            // I2C Control struct

IMU_t *get_mpu6050_imu(void) {
    static IMU_t mpu6050_imu = {
        .init = initialize,
        .id = get_device_id,
        .acc_x = get_acceleration_x,
        .acc_y = get_acceleration_y,
        .acc_z = get_acceleration_z,
        .gyro_x = get_rotation_x,
        .gyro_y = get_rotation_y,
        .gyro_z = get_rotation_z
    };
    return &mpu6050_imu;
}

// -----------------------------------------------------------------------------

static const char* getIMUErrorText(IMU_Fails_t error) {
    switch(error) {
        case IMU_Ok:
            return "IMU_Ok";
        case IMU_Config_Error:
            return "IMU_Config_Error";
        case IMU_COMM_BUS_ERROR:
            return "IMU_COMM_BUS_ERROR";
        case IMU_Read_Timeout:
            return "IMU_Read_Timeout";
        case IMU_Busy_Timeout:
            return "IMU_Busy_Timeout";
        default:
            return "Unknown error";
    }
}

// -----------------------------------------------------------------------------

IMU_Fails_t NO_OPT
initialize(void) {
    log_message(INFO, MPU6050, "Setting up MPU6050.");

    i2c_setup_peripheral();
    setup_reset_pin();
    hard_reset();
    
    IMU_Fails_t status = i2c_configure(&i2c, I2C1, MPU6050_DEFAULT_ADDRESS, 1000);
    if(status) {
        log_message_with_error(ERROR, MPU6050, "Fail to setup I2C",
          getIMUErrorText(IMU_COMM_BUS_ERROR));
        
        return IMU_COMM_BUS_ERROR;
    }

    log_message(DEBUG, I2C_BUS,"Setting clock source!");
    status = set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    if(status) {
        log_message_with_error(ERROR, MPU6050, "Fail to set clock source. Error: ",
          getIMUErrorText(status));
        
        return IMU_Config_Error;
    }

    log_message(DEBUG, I2C_BUS,"Setting gyro scale range!");
    status = set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
    if(status){
        log_message_with_error(ERROR, MPU6050, "Fail to set scale gyro range. Err: ", 
          getIMUErrorText(status));
        return IMU_Config_Error;
    }

    log_message(DEBUG, I2C_BUS,"Setting accelerometer scale range!");
    status = set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
    if(status){
        log_message_with_error(ERROR, MPU6050, "Fail to set scale accel range. Err: ", 
          getIMUErrorText(status));
        return IMU_Config_Error;
    }

    log_message(DEBUG, I2C_BUS,"Disabling sleep enable!");
    status = set_sleep_enabled(false);
    if(status){
        log_message_with_error(ERROR, MPU6050, "Fail to set sleep enable. Err: ", 
          getIMUErrorText(status));
        return IMU_Config_Error;
    }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

void
setup_reset_pin(void) {
    log_message(INFO, MPU6050, "Setting up reset pin.");
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Set GPIO10 (in GPIO port A) to 'output push-pull'. */
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_2_MHZ,
              GPIO_CNF_OUTPUT_PUSHPULL,GPIO10);
}

// -----------------------------------------------------------------------------

void
hard_reset(void) {
    log_message(INFO, MPU6050, "Hard reseting.");

    gpio_clear(GPIOA,GPIO10);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set(GPIOA,GPIO10); // enable ON
}

// -----------------------------------------------------------------------------

bool
test_connection(void) {
    return get_device_id() == MPU6050_DEFAULT_ADDRESS;
}

// -----------------------------------------------------------------------------

uint8_t NO_OPT
get_device_id(void) {
    uint8_t buffer = 0;
    i2c_read_byte(&i2c, MPU6050_RA_WHO_AM_I, &buffer);

    return buffer;
}

// -----------------------------------------------------------------------------

void set_device_id(uint8_t id) {
    i2c_write_bits(&i2c, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// -----------------------------------------------------------------------------

void NO_OPT
soft_reset(void) {
    i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

// -----------------------------------------------------------------------------

IMU_Fails_t NO_OPT
set_clock_source(uint8_t source) {

    if(i2c_write_bits(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, 
      MPU6050_PWR1_CLKSEL_LENGTH, source)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails_t NO_OPT
set_full_scale_gyro_range(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, 
      MPU6050_GCONFIG_FS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails_t NO_OPT
set_full_scale_accel_range(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, 
      MPU6050_ACONFIG_AFS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

IMU_Fails_t NO_OPT
set_sleep_enabled(bool enabled) {
    if(i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 
      enabled)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_Ok;
}

// -----------------------------------------------------------------------------

int16_t get_acceleration_x(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_XOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t get_acceleration_y(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_YOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t get_acceleration_z(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_ACCEL_ZOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t get_rotation_x(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_XOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t get_rotation_y(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_YOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// -----------------------------------------------------------------------------

int16_t get_rotation_z(void) {
    uint8_t buffer[2];
    i2c_read_bytes(&i2c, (MPU6050_RA_GYRO_ZOUT_H), buffer, 2);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}