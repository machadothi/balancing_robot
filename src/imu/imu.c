/**
 * @file imu.c
 * @brief IMU service: sampling task and latest-sample mailbox
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <math.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"
#include "board_config.h"
#include "app/module.h"
#include "imu/imu.h"
#if IMU_SENSOR_QMI8658
#include "imu/qmi8658.h"
#else
#include "imu/mpu6050.h"
#endif // IMU_SENSOR_QMI8658
#include "log/log.h"
#if CONSOLE_ANY
#include "cmd/at_cmd.h"
#include "util/fmt.h"
#endif // CONSOLE_ANY

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** The sensor in use: the only line that names a specific driver */
#if IMU_SENSOR_QMI8658
static const IMU_Ops_t *const sensor = &qmi8658_ops;
#else
static const IMU_Ops_t *const sensor = &mpu6050_ops;
#endif // IMU_SENSOR_QMI8658

/** Holds one sample (IMU_QUEUE_SIZE 1): overwritten, never backlogged */
static QueueHandle_t samples = NULL;

/** Start-up gyro calibration: average this many samples (1 s at 10 ms) */
#define GYRO_CAL_SAMPLES            100

/** Any axis varying more than this (deg/s) during calibration means the robot moved */
#define GYRO_CAL_MAX_SPREAD_DPS     2.0f

/** Measured gyro bias (deg/s), subtracted from every sample */
static volatile float gyro_bias[3];

/* ==========================================================================
 * Gyro calibration
 * ========================================================================== */

/**
 * @brief Average the gyro while the robot is still
 *
 * Retries while the robot moves (any axis spread above the limit). Samples are
 * not published meanwhile: the control loop sees an IMU stall and keeps the
 * motors off.
 */
static void calibrate_gyro(TickType_t *last_wake_time) {
    for (;;) {
        float sum[3] = { 0 }, low[3] = { 0 }, high[3] = { 0 };
        int count = 0;

        while (count < GYRO_CAL_SAMPLES) {
            IMU_Data_t s;
            vTaskDelayUntil(last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_RATE_MS));
            if (sensor->read(&s) != IMU_OK) {
                continue;
            }
            const float g[3] = { s.gyro_x, s.gyro_y, s.gyro_z };
            for (int a = 0; a < 3; a++) {
                sum[a] += g[a];
                low[a] = (count == 0 || g[a] < low[a]) ? g[a] : low[a];
                high[a] = (count == 0 || g[a] > high[a]) ? g[a] : high[a];
            }
            count++;
        }

        bool still = true;
        for (int a = 0; a < 3; a++) {
            still = still && (high[a] - low[a]) <= GYRO_CAL_MAX_SPREAD_DPS;
        }
        if (still) {
            for (int a = 0; a < 3; a++) {
                gyro_bias[a] = sum[a] / GYRO_CAL_SAMPLES;
            }
            log_message(LOG_INFO, IMU_TASK, "Gyro bias calibrated");
            return;
        }
        log_message(LOG_WARN, IMU_TASK, "Robot moved during gyro calibration, retrying");
    }
}

#if CONSOLE_ANY
static AT_Result_t query_gyro_bias(char *value, size_t size) {
    char x[16], y[16], z[16];
    snprintf(value, size, "%s,%s,%s", fmt_fixed(x, sizeof(x), gyro_bias[0], 3),
             fmt_fixed(y, sizeof(y), gyro_bias[1], 3), fmt_fixed(z, sizeof(z), gyro_bias[2], 3));
    return AT_OK;
}

static const AT_Command_Def_t imu_commands[] = {
    { .name = "GYROBIAS", .query = query_gyro_bias,
      .help = AT_HELP("Gyro bias x,y,z (deg/s) measured at start-up") },
};
#endif // CONSOLE_ANY

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void imu_queue_init(void) {
    samples = xQueueCreate(IMU_QUEUE_SIZE, sizeof(IMU_Data_t));
#if CONSOLE_ANY
    at_cmd_register(imu_commands, sizeof(imu_commands) / sizeof(imu_commands[0]));
#endif // CONSOLE_ANY
}

IMU_Tilt_t imu_tilt(const IMU_Data_t *sample) {
    IMU_Tilt_t tilt = {
        .acc_deg = atan2f(BOARD_TILT_ACC_NUM(sample), BOARD_TILT_ACC_DEN(sample)) * (180.0f / 3.14159265f),
        .rate_dps = BOARD_TILT_RATE(sample),
    };
    return tilt;
}

bool imu_wait_sample(IMU_Data_t *out, TickType_t timeout) {
    return xQueueReceive(samples, out, timeout) == pdPASS;
}

void imu_task(void *args) {
    (void)args;

    IMU_Status_t status;
    while ((status = sensor->init()) != IMU_OK) {
        /* IMU_Status_t: 1 config (no ACK), 2 bus (lines stuck), 3/4 timeouts */
        log_message_with_int(LOG_ERROR, IMU_TASK, "IMU init failed, retrying; status", (int)status);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    log_message(LOG_INFO, IMU_TASK, "IMU initialized");

    IMU_Data_t sample;

    /* Read once: vTaskDelayUntil then keeps a fixed period instead of period + work time */
    TickType_t last_wake_time = xTaskGetTickCount();

    /* Keep the robot still for the first second after power-on */
    calibrate_gyro(&last_wake_time);

    for (;;) {
        if (sensor->read(&sample) == IMU_OK) {
            sample.gyro_x -= gyro_bias[0];
            sample.gyro_y -= gyro_bias[1];
            sample.gyro_z -= gyro_bias[2];
            /* The controller always gets the latest measurement */
            xQueueOverwrite(samples, &sample);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_RATE_MS));
    }
}

APP_MODULE(imu_module) = {
    .name = "IMU",
    .init = imu_queue_init,
    .task = imu_task,
    .stack = 192,
    .priority = APP_PRIORITY_CONTROL,
};
