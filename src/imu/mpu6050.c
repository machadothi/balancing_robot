#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "config.h"
#include "mpu6050.h"
#include "board_config.h"
#include "drivers/gpio_compat.h"
#include "drivers/i2c.h"
#include "log/log.h"

/* MPU6050 reads 14 bytes: AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ (2 bytes each) */
#define MPU6050_DATA_SIZE   14

/* I2C Control struct - non-static for ISR access from interrupts.c */
I2C_Control_t i2c;
static SemaphoreHandle_t i2c_transfer_sem;      /* DMA transfer complete semaphore */
static volatile uint8_t dma_buffer[MPU6050_DATA_SIZE];  /* DMA receive buffer */
static volatile bool dma_transfer_ok = false;   /* Transfer status */

/* Cached sensor values (updated after each DMA transfer) */
static volatile int16_t cached_acc_x, cached_acc_y, cached_acc_z;
static volatile int16_t cached_gyro_x, cached_gyro_y, cached_gyro_z;

IMU_Driver_t *mpu6050_get_driver(void) {
    static IMU_Driver_t mpu6050_imu = {
        .init = mpu6050_init,
        .id = mpu6050_get_device_id,
        .read_all = mpu6050_read_all_dma,
        .acc_x = mpu6050_get_acceleration_x,
        .acc_y = mpu6050_get_acceleration_y,
        .acc_z = mpu6050_get_acceleration_z,
        .gyro_x = mpu6050_get_rotation_x,
        .gyro_y = mpu6050_get_rotation_y,
        .gyro_z = mpu6050_get_rotation_z
    };
    return &mpu6050_imu;
}

// -----------------------------------------------------------------------------

static const char* getIMUErrorText(IMU_Status_t error) {
    switch(error) {
        case IMU_OK:
            return "IMU_OK";
        case IMU_CONFIG_ERROR:
            return "IMU_CONFIG_ERROR";
        case IMU_COMM_BUS_ERROR:
            return "IMU_COMM_BUS_ERROR";
        case IMU_READ_TIMEOUT:
            return "IMU_READ_TIMEOUT";
        case IMU_BUSY_TIMEOUT:
            return "IMU_BUSY_TIMEOUT";
        default:
            return "Unknown error";
    }
}

// -----------------------------------------------------------------------------

/**
 * @brief DMA transfer complete callback
 * 
 * Called from ISR context when DMA transfer finishes.
 * Parses the raw buffer into cached sensor values and signals the semaphore.
 */
static void mpu6050_dma_callback(I2C_Control_t *dev, I2C_Fails_t result) {
    (void)dev;
    BaseType_t higher_priority_woken = pdFALSE;
    
    if (result == I2C_Ok) {
        /* Parse DMA buffer into cached values (big-endian) */
        cached_acc_x  = ((int16_t)dma_buffer[0] << 8) | dma_buffer[1];
        cached_acc_y  = ((int16_t)dma_buffer[2] << 8) | dma_buffer[3];
        cached_acc_z  = ((int16_t)dma_buffer[4] << 8) | dma_buffer[5];
        /* Skip temperature (bytes 6-7) */
        cached_gyro_x = ((int16_t)dma_buffer[8] << 8) | dma_buffer[9];
        cached_gyro_y = ((int16_t)dma_buffer[10] << 8) | dma_buffer[11];
        cached_gyro_z = ((int16_t)dma_buffer[12] << 8) | dma_buffer[13];
        dma_transfer_ok = true;
    } else {
        dma_transfer_ok = false;
    }
    
    /* Signal the waiting task */
    xSemaphoreGiveFromISR(i2c_transfer_sem, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_init(void) {
    log_message(LOG_INFO, MPU6050, "Setting up MPU6050.");

    i2c_setup_peripheral();
    mpu6050_setup_reset_pin();
    mpu6050_hard_reset();
    
    /* Create binary semaphore for DMA transfer synchronization */
    i2c_transfer_sem = xSemaphoreCreateBinary();
    if (i2c_transfer_sem == NULL) {
        log_message(LOG_ERROR, MPU6050, "Failed to create I2C semaphore");
        return IMU_CONFIG_ERROR;
    }
    
    /* Configure I2C (polling mode for initialization commands) */
    I2C_Fails_t i2c_status = i2c_configure(&i2c, BOARD_I2C, MPU6050_DEFAULT_ADDRESS, 1000);
    if(i2c_status) {
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to setup I2C",
          getIMUErrorText(IMU_COMM_BUS_ERROR));
        
        return IMU_COMM_BUS_ERROR;
    }
    
    /* Initialize DMA for sensor reads (priority 5) */
    /* 11 << 4 = 0xB0: the most urgent priority allowed to call FreeRTOS FromISR APIs */
    i2c_init_dma(&i2c, 11);
    i2c.callback = mpu6050_dma_callback;

    log_message(LOG_DEBUG, I2C_BUS,"Setting clock source!");
    IMU_Status_t status = mpu6050_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    if(status) {
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to set clock source. Error: ",
          getIMUErrorText(status));
        
        return IMU_CONFIG_ERROR;
    }

    log_message(LOG_DEBUG, I2C_BUS,"Setting gyro scale range!");
    status = mpu6050_set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
    if(status){
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to set scale gyro range. Err: ", 
          getIMUErrorText(status));
        return IMU_CONFIG_ERROR;
    }

    log_message(LOG_DEBUG, I2C_BUS,"Setting accelerometer scale range!");
    status = mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
    if(status){
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to set scale accel range. Err: ", 
          getIMUErrorText(status));
        return IMU_CONFIG_ERROR;
    }

    /* Without it, 50-250 Hz motor vibration aliases into the 100 Hz samples */
    log_message(LOG_DEBUG, I2C_BUS,"Setting digital low-pass filter!");
    status = mpu6050_set_dlpf_mode(IMU_DLPF_MODE);
    if(status){
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to set low-pass filter. Err: ",
          getIMUErrorText(status));
        return IMU_CONFIG_ERROR;
    }

    log_message(LOG_DEBUG, I2C_BUS,"Disabling sleep enable!");
    status = mpu6050_set_sleep_enabled(false);
    if(status){
        log_message_with_error(LOG_ERROR, MPU6050, "Fail to set sleep enable. Err: ", 
          getIMUErrorText(status));
        return IMU_CONFIG_ERROR;
    }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

void mpu6050_setup_reset_pin(void) {
#ifdef BOARD_IMU_RESET_PORT
    log_message(LOG_INFO, MPU6050, "Setting up reset pin.");
    rcc_periph_clock_enable(BOARD_IMU_RESET_PORT_RCC);

    gpio_compat_output(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN, false);
#endif // BOARD_IMU_RESET_PORT
}

// -----------------------------------------------------------------------------

void mpu6050_hard_reset(void) {
#ifdef BOARD_IMU_RESET_PORT
    log_message(LOG_INFO, MPU6050, "Hard reseting.");

    gpio_clear(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN); // enable ON
#endif // BOARD_IMU_RESET_PORT
}

// -----------------------------------------------------------------------------

bool
mpu6050_test_connection(void) {
    return mpu6050_get_device_id() == MPU6050_DEFAULT_ADDRESS;
}

// -----------------------------------------------------------------------------

uint8_t
mpu6050_get_device_id(void) {
    uint8_t buffer = 0;
    i2c_read_byte(&i2c, MPU6050_RA_WHO_AM_I, &buffer);

    return buffer;
}

// -----------------------------------------------------------------------------

void mpu6050_set_device_id(uint8_t id) {
    i2c_write_bits(&i2c, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// -----------------------------------------------------------------------------

void
mpu6050_soft_reset(void) {
    i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_set_clock_source(uint8_t source) {

    if(i2c_write_bits(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, 
      MPU6050_PWR1_CLKSEL_LENGTH, source)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_set_full_scale_gyro_range(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, 
      MPU6050_GCONFIG_FS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_set_full_scale_accel_range(uint8_t range) {
    if(i2c_write_bits(&i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, 
      MPU6050_ACONFIG_AFS_SEL_LENGTH, range)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_set_dlpf_mode(uint8_t mode) {
    if(i2c_write_bits(&i2c, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT,
      MPU6050_CFG_DLPF_CFG_LENGTH, mode)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

IMU_Status_t
mpu6050_set_sleep_enabled(bool enabled) {
    if(i2c_write_bit(&i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 
      enabled)) {
        return IMU_COMM_BUS_ERROR;
      }

    return IMU_OK;
}

// -----------------------------------------------------------------------------

/**
 * @brief Trigger a DMA read of all sensor data (14 bytes)
 * 
 * Reads accelerometer (3 axes), temperature, and gyroscope (3 axes) in one
 * DMA transfer. Blocks until transfer completes or timeout.
 * 
 * @return IMU_Status_t IMU_OK on success, error code otherwise
 */
IMU_Status_t
mpu6050_read_all_dma(void) {
    /* Start DMA read of all 14 bytes starting from ACCEL_XOUT_H */
    I2C_Fails_t status = i2c_read_reg_dma(&i2c, MPU6050_RA_ACCEL_XOUT_H,
                                           (uint8_t*)dma_buffer, MPU6050_DATA_SIZE,
                                           mpu6050_dma_callback);
    if (status != I2C_Ok) {
        return IMU_COMM_BUS_ERROR;
    }
    
    /* Wait for DMA transfer to complete (timeout 100ms) */
    if (xSemaphoreTake(i2c_transfer_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        i2c_abort(&i2c);
        return IMU_READ_TIMEOUT;
    }
    
    /* Check transfer result */
    if (!dma_transfer_ok) {
        return IMU_COMM_BUS_ERROR;
    }
    
    return IMU_OK;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_acceleration_x(void) {
    return cached_acc_x;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_acceleration_y(void) {
    return cached_acc_y;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_acceleration_z(void) {
    return cached_acc_z;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_rotation_x(void) {
    return cached_gyro_x;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_rotation_y(void) {
    return cached_gyro_y;
}

// -----------------------------------------------------------------------------

int16_t mpu6050_get_rotation_z(void) {
    return cached_gyro_z;
}