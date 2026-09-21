/**
 * @file mpu6050.c
 * @brief MPU-6050 driver: register setup over polled I2C, samples over DMA
 *
 * Everything is static except mpu6050_ops. Errors are returned, not logged:
 * the IMU task decides what to report.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "config.h"
#include "board_config.h"
#include "drivers/gpio_compat.h"
#include "drivers/i2c.h"
#include "imu/mpu6050.h"
#include "imu/mpu6050_regs.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** AccX, AccY, AccZ, Temp, GyroX, GyroY, GyroZ: 2 bytes each, big-endian */
#define MPU6050_DATA_SIZE           14

/** Polled I2C timeout during setup (ticks) */
#define MPU6050_I2C_TIMEOUT         1000

/** Longest wait for a DMA burst to complete */
#define MPU6050_READ_TIMEOUT_MS     100

/** 11 << 4 = 0xB0: the most urgent priority allowed to call FreeRTOS FromISR APIs */
#define MPU6050_DMA_IRQ_PRIORITY    11

/** One register bit field written during setup */
typedef struct {
    uint8_t reg;
    uint8_t bit;
    uint8_t length;
    uint8_t value;
} MPU6050_Field_t;

static const MPU6050_Field_t setup_fields[] = {
    { MPU6050_RA_PWR_MGMT_1,   MPU6050_PWR1_CLKSEL_BIT,     MPU6050_PWR1_CLKSEL_LENGTH,     MPU6050_CLOCK_PLL_XGYRO },
    { MPU6050_RA_GYRO_CONFIG,  MPU6050_GCONFIG_FS_SEL_BIT,  MPU6050_GCONFIG_FS_SEL_LENGTH,  MPU6050_GYRO_FS_250 },
    { MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2 },
    /* Without it, 50-250 Hz motor vibration aliases into the 100 Hz samples */
    { MPU6050_RA_CONFIG,       MPU6050_CFG_DLPF_CFG_BIT,    MPU6050_CFG_DLPF_CFG_LENGTH,    IMU_DLPF_MODE },
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

static I2C_Control_t *const bus = &i2c_board_bus;

/** Given by the DMA completion callback */
static SemaphoreHandle_t transfer_done = NULL;
static volatile bool transfer_ok = false;

/** Filled by DMA; read only after transfer_done */
static uint8_t burst[MPU6050_DATA_SIZE];

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/** Runs in the DMA interrupt */
static void transfer_callback(I2C_Control_t *dev, I2C_Fails_t result) {
    (void)dev;
    BaseType_t higher_priority_woken = pdFALSE;

    transfer_ok = (result == I2C_Ok);
    xSemaphoreGiveFromISR(transfer_done, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

/** Power-cycles the sensor if the board switches its supply */
static void hard_reset(void) {
#ifdef BOARD_IMU_RESET_PORT
    rcc_periph_clock_enable(BOARD_IMU_RESET_PORT_RCC);
    gpio_compat_output(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN, false);

    gpio_clear(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set(BOARD_IMU_RESET_PORT, BOARD_IMU_RESET_PIN);
#endif // BOARD_IMU_RESET_PORT
}

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* ==========================================================================
 * IMU_Ops_t
 * ========================================================================== */

/** Safe to call again after a failure (the IMU task retries) */
static IMU_Status_t mpu6050_init(void) {
    i2c_setup_peripheral();
    hard_reset();

    if (transfer_done == NULL) {
        transfer_done = xSemaphoreCreateBinary();
        if (transfer_done == NULL) {
            return IMU_CONFIG_ERROR;
        }
    }

    if (i2c_configure(bus, BOARD_I2C, MPU6050_DEFAULT_ADDRESS, MPU6050_I2C_TIMEOUT) != I2C_Ok) {
        return IMU_COMM_BUS_ERROR;
    }


    i2c_init_dma(bus, MPU6050_DMA_IRQ_PRIORITY);
    bus->callback = transfer_callback;

    for (size_t i = 0; i < sizeof(setup_fields) / sizeof(setup_fields[0]); i++) {
        const MPU6050_Field_t *f = &setup_fields[i];
        if (i2c_write_bits(bus, f->reg, f->bit, f->length, f->value) != I2C_Ok) {
            return IMU_CONFIG_ERROR;
        }
    }

    /* Last: the sensor starts sampling with the final configuration */
    if (i2c_write_bit(bus, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false) != I2C_Ok) {
        return IMU_CONFIG_ERROR;
    }

    return IMU_OK;
}

static IMU_Status_t mpu6050_read(IMU_Data_t *out) {
    /* Drop a completion that arrived after an earlier read had timed out */
    (void)xSemaphoreTake(transfer_done, 0);

    if (i2c_read_reg_dma(bus, MPU6050_RA_ACCEL_XOUT_H, burst, MPU6050_DATA_SIZE,
                         transfer_callback) != I2C_Ok) {
        return IMU_COMM_BUS_ERROR;
    }

    if (xSemaphoreTake(transfer_done, pdMS_TO_TICKS(MPU6050_READ_TIMEOUT_MS)) != pdTRUE) {
        i2c_abort(bus);
        return IMU_READ_TIMEOUT;
    }

    if (!transfer_ok) {
        return IMU_COMM_BUS_ERROR;
    }

    out->acc_x  = be16(&burst[0])  / ACC_SENS_SCALE_FACTOR;
    out->acc_y  = be16(&burst[2])  / ACC_SENS_SCALE_FACTOR;
    out->acc_z  = be16(&burst[4])  / ACC_SENS_SCALE_FACTOR;
    /* bytes 6-7: temperature, unused */
    out->gyro_x = be16(&burst[8])  / GYRO_SENS_SCALE_FACTOR;
    out->gyro_y = be16(&burst[10]) / GYRO_SENS_SCALE_FACTOR;
    out->gyro_z = be16(&burst[12]) / GYRO_SENS_SCALE_FACTOR;

    return IMU_OK;
}

const IMU_Ops_t mpu6050_ops = {
    .name = "MPU-6050",
    .init = mpu6050_init,
    .read = mpu6050_read,
};
