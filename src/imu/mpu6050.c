#include "mpu6050.h"
#include "../communication/i2c.h"


static I2C_Control i2c;            // I2C Control struct

void initialize(void) {
    i2c_configure(&i2c, I2C1, MPU6050_DEFAULT_ADDRESS, 1000);
    // i2c_configure(&i2c, I2C1, MPU6050_ADDRESS_AD0_HIGH, 1000);

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

uint8_t getDeviceID(void) {
    uint8_t buffer = 0;
    readByte(&i2c, MPU6050_RA_WHO_AM_I, &buffer);
    return buffer;
}

// -----------------------------------------------------------------------------

void setDeviceID(uint8_t id) {
    writeBits(&i2c, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// -----------------------------------------------------------------------------

void setClockSource(uint8_t source) {
    writeBits(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

// -----------------------------------------------------------------------------

void setFullScaleGyroRange(uint8_t range) {
    writeBits(&i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// -----------------------------------------------------------------------------

void setFullScaleAccelRange(uint8_t range) {
    writeBits(&i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

// -----------------------------------------------------------------------------

void setSleepEnabled(bool enabled) {
    writeBit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}