#include "mpu6050.h"
#include "../communication/i2c.h"

#define NO_OPT __attribute__((optimize("O0")))

static I2C_Control i2c;            // I2C Control struct

void NO_OPT initialize(void) {
    i2c_configure(&i2c, I2C1, MPU6050_DEFAULT_ADDRESS);

    // imu_reset();
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false);
}

// -----------------------------------------------------------------------------

bool testConnection(void) {
    return getDeviceID() == 0x34;
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
imu_reset(void) {
    i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

// -----------------------------------------------------------------------------

void NO_OPT
setClockSource(uint8_t source) {

    i2c_write_bits(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

// -----------------------------------------------------------------------------

void NO_OPT
setFullScaleGyroRange(uint8_t range) {
    i2c_write_bits(&i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// -----------------------------------------------------------------------------

void NO_OPT
setFullScaleAccelRange(uint8_t range) {
    i2c_write_bits(&i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

// -----------------------------------------------------------------------------

void NO_OPT
setSleepEnabled(bool enabled) {
    i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}