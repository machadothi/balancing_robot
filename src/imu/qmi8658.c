/**
 * @file qmi8658.c
 * @brief QMI8658 driver: register setup over polled I2C, samples over DMA
 *
 * Found at address 0x6A on the Hiwonder F407 board (I2C2, PB10/PB11), where
 * the vendor firmware's names still say MPU-6050. Everything is static
 * except qmi8658_ops. Errors are returned, not logged.
 */

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "config.h"
#include "board_config.h"
#include "drivers/i2c.h"
#include "imu/qmi8658.h"

/* ==========================================================================
 * Registers (QMI8658 datasheet)
 * ========================================================================== */

#define QMI8658_ADDRESS         0x6A
#define QMI8658_WHO_AM_I_VALUE  0x05

#define REG_WHO_AM_I            0x00
#define REG_CTRL1               0x02    /**< Serial interface */
#define REG_CTRL2               0x03    /**< Accelerometer range and rate */
#define REG_CTRL3               0x04    /**< Gyroscope range and rate */
#define REG_CTRL5               0x06    /**< Low-pass filters */
#define REG_CTRL7               0x08    /**< Sensor enable */
#define REG_AX_L                0x35    /**< AX, AY, AZ, GX, GY, GZ: 16-bit little-endian */
#define REG_RESET               0x60

#define CTRL1_ADDR_AUTO_INC     0x40    /**< Burst reads walk the registers; little-endian */
#define ODR_224HZ               0x05
#define CTRL2_ACC_2G            (0x0 << 4)
#define CTRL3_GYRO_256DPS       (0x4 << 4)
/** Both filters on, cut-off 13.37 % of the output rate: ~30 Hz at 224 Hz */
#define CTRL5_LPF_30HZ          ((0x3 << 5) | (1 << 4) | (0x3 << 1) | (1 << 0))
#define CTRL7_ACC_GYRO_ON       0x03
#define RESET_SOFT              0xB0

/** Scale factors for ±2 g and ±256 °/s */
#define ACC_LSB_PER_G           16384.0f
#define GYRO_LSB_PER_DPS        128.0f

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

#define QMI8658_DATA_SIZE           12
#define QMI8658_I2C_TIMEOUT         1000
#define QMI8658_READ_TIMEOUT_MS     100
#define QMI8658_RESET_MS            20
#define QMI8658_STARTUP_MS          50

/** 11 << 4 = 0xB0: the most urgent priority allowed to call FreeRTOS FromISR APIs */
#define QMI8658_DMA_IRQ_PRIORITY    11

typedef struct {
    uint8_t reg;
    uint8_t value;
} QMI8658_Setting_t;

static const QMI8658_Setting_t setup[] = {
    { REG_CTRL1, CTRL1_ADDR_AUTO_INC },
    { REG_CTRL2, CTRL2_ACC_2G | ODR_224HZ },
    { REG_CTRL3, CTRL3_GYRO_256DPS | ODR_224HZ },
    { REG_CTRL5, CTRL5_LPF_30HZ },
    /* Last: the sensor starts sampling with the final configuration */
    { REG_CTRL7, CTRL7_ACC_GYRO_ON },
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

static I2C_Control_t *const bus = &i2c_board_bus;

static SemaphoreHandle_t transfer_done = NULL;
static volatile bool transfer_ok = false;

/** Filled by DMA; read only after transfer_done */
static uint8_t burst[QMI8658_DATA_SIZE];

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

static int16_t le16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* ==========================================================================
 * IMU_Ops_t
 * ========================================================================== */

/** Safe to call again after a failure (the IMU task retries) */
static IMU_Status_t qmi8658_init(void) {
    i2c_setup_peripheral();

    if (transfer_done == NULL) {
        transfer_done = xSemaphoreCreateBinary();
        if (transfer_done == NULL) {
            return IMU_CONFIG_ERROR;
        }
    }

    if (i2c_configure(bus, BOARD_I2C, QMI8658_ADDRESS, QMI8658_I2C_TIMEOUT) != I2C_Ok) {
        return IMU_COMM_BUS_ERROR;
    }

    uint8_t who = 0;
    if (i2c_read_byte(bus, REG_WHO_AM_I, &who) != I2C_Ok || who != QMI8658_WHO_AM_I_VALUE) {
        return IMU_CONFIG_ERROR;
    }

    (void)i2c_write_byte(bus, REG_RESET, RESET_SOFT);
    vTaskDelay(pdMS_TO_TICKS(QMI8658_RESET_MS));

    i2c_init_dma(bus, QMI8658_DMA_IRQ_PRIORITY);
    bus->callback = transfer_callback;

    for (size_t i = 0; i < sizeof(setup) / sizeof(setup[0]); i++) {
        if (i2c_write_byte(bus, setup[i].reg, setup[i].value) != I2C_Ok) {
            return IMU_CONFIG_ERROR;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(QMI8658_STARTUP_MS));
    return IMU_OK;
}

static IMU_Status_t qmi8658_read(IMU_Data_t *out) {
    /* Drop a completion that arrived after an earlier read had timed out */
    (void)xSemaphoreTake(transfer_done, 0);

    if (i2c_read_reg_dma(bus, REG_AX_L, burst, QMI8658_DATA_SIZE, transfer_callback) != I2C_Ok) {
        return IMU_COMM_BUS_ERROR;
    }

    if (xSemaphoreTake(transfer_done, pdMS_TO_TICKS(QMI8658_READ_TIMEOUT_MS)) != pdTRUE) {
        i2c_abort(bus);
        return IMU_READ_TIMEOUT;
    }

    if (!transfer_ok) {
        return IMU_COMM_BUS_ERROR;
    }

    out->acc_x  = le16(&burst[0])  / ACC_LSB_PER_G;
    out->acc_y  = le16(&burst[2])  / ACC_LSB_PER_G;
    out->acc_z  = le16(&burst[4])  / ACC_LSB_PER_G;
    out->gyro_x = le16(&burst[6])  / GYRO_LSB_PER_DPS;
    out->gyro_y = le16(&burst[8])  / GYRO_LSB_PER_DPS;
    out->gyro_z = le16(&burst[10]) / GYRO_LSB_PER_DPS;

    return IMU_OK;
}

const IMU_Ops_t qmi8658_ops = {
    .name = "QMI8658",
    .init = qmi8658_init,
    .read = qmi8658_read,
};
