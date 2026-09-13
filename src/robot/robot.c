/**
 * @file robot.c
 * @brief Balance control task: sensor fusion, PID, safety states
 *
 * One pass per IMU sample: estimate the tilt, update the shared state, run the
 * balance controller while enabled, and hand a telemetry record to the logger.
 * The AT commands operating on this state live in robot_commands.c.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "config.h"
#include "app/module.h"
#include "board/board.h"
#include "control/mixer.h"
#include "control/pid.h"
#include "filter/filter.h"
#include "imu/imu.h"
#include "motor/motor.h"
#include "robot/robot.h"
#include "robot/robot_internal.h"
#if TELEMETRY
#include "telemetry/telemetry.h"
#endif // TELEMETRY

/* ==========================================================================
 * Constants
 * ========================================================================== */

#define RAD_TO_DEG              (180.0f / 3.14159265f)

/** Balance PID gains at startup and after AT+DEFAULT */
#define ROBOT_DEFAULT_KP        25.0f
#define ROBOT_DEFAULT_KI        0.5f
#define ROBOT_DEFAULT_KD        0.8f

/** Clamp on the accumulated angle error (deg x s) */
#define PID_INTEGRAL_LIMIT      100.0f

/** Setpoint angle for balance (degrees from vertical) */
#define BALANCE_SETPOINT        0.0f

/** Beyond this tilt recovery is impossible: stop instead (degrees) */
#define MAX_TILT_ANGLE          45.0f

/** Reported as BALANCED below this tilt (degrees) */
#define BALANCED_TILT_ANGLE     5.0f

/** Minimum non-zero motor command, to overcome static friction */
#define MOTOR_DEADBAND          20

/* ==========================================================================
 * Shared State
 * ========================================================================== */

Robot_t robot = {
    .pid = {
        .kp = ROBOT_DEFAULT_KP,
        .ki = ROBOT_DEFAULT_KI,
        .kd = ROBOT_DEFAULT_KD,
        .integral_limit = PID_INTEGRAL_LIMIT,
        .output_limit = (float)MOTOR_COMMAND_MAX,
    },
    .pid_enabled = true,
};

/** Guards `robot` and motor commands against the AT handlers (UART RX task) */
static SemaphoreHandle_t state_mutex = NULL;

/* ==========================================================================
 * Attitude filters: all run every sample (telemetry compares them), one steers
 * ========================================================================== */

typedef enum {
    FILTER_COMPLEMENTARY = 0,
    FILTER_KALMAN,
    FILTER_COUNT
} Filter_Id_t;

#if ATTITUDE_FILTER_KALMAN
#define CONTROL_FILTER          FILTER_KALMAN
#else
#define CONTROL_FILTER          FILTER_COMPLEMENTARY
#endif // ATTITUDE_FILTER_KALMAN

/* ==========================================================================
 * State Transitions (caller holds the lock)
 * ========================================================================== */

void robot_lock(void) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
}

void robot_unlock(void) {
    xSemaphoreGive(state_mutex);
}

void robot_enable(void) {
    pid_reset(&robot.pid);
    robot.motors_enabled = true;
    motor_standby(false);
}

void robot_disable(void) {
    robot.motors_enabled = false;
    motor_set(MOTOR_LEFT, 0);
    motor_set(MOTOR_RIGHT, 0);
    motor_standby(true);
    pid_reset(&robot.pid);
}

void robot_restore_defaults(void) {
    robot.pid.kp = ROBOT_DEFAULT_KP;
    robot.pid.ki = ROBOT_DEFAULT_KI;
    robot.pid.kd = ROBOT_DEFAULT_KD;
    robot.target_velocity = 0.0f;
    robot.turn_rate = 0.0f;
}

/* ==========================================================================
 * Control
 * ========================================================================== */

/**
 * @brief Tilt from gravity
 *
 * The IMU is mounted with its X axis vertical (ax is about -1 g upright), so
 * upright reads about 90 degrees.
 */
static float calc_angle_from_accel(const IMU_Data_t *data) {
    return atan2f(data->acc_y, -data->acc_x) * RAD_TO_DEG;
}

/** One balance update: safety cut-off, PID, mixing, motors */
static void robot_balance_step(float tilt) {
    if (fabsf(tilt) > MAX_TILT_ANGLE) {
        robot_disable();
        return;
    }

    float output = pid_update(&robot.pid, BALANCE_SETPOINT - tilt, IMU_SAMPLE_RATE_S);
    Mixer_Output_t wheels = mixer_mix(output, robot.turn_rate, MOTOR_COMMAND_MAX, MOTOR_DEADBAND);

    motor_set(MOTOR_LEFT, wheels.left);
    motor_set(MOTOR_RIGHT, wheels.right);
}

/* ==========================================================================
 * Task
 * ========================================================================== */

/** Before the scheduler: the lock exists before any AT handler can take it */
static void robot_init(void) {
    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex != NULL);

#if CONSOLE_ANY
    robot_commands_register();
#endif // CONSOLE_ANY
}

void robot_task(void *args) {
    (void)args;

    KalmanFilter_t kalman;
    ComplementaryFilter_t complementary;
    kalman_init(&kalman);
    complementary_init(&complementary);

    AttitudeFilter_t filters[FILTER_COUNT] = {
        [FILTER_COMPLEMENTARY] = complementary_filter_interface(&complementary),
        [FILTER_KALMAN] = kalman_filter_interface(&kalman),
    };
    float angles[FILTER_COUNT];
    bool filters_seeded = false;

#if AUTO_ENABLE
    bool auto_enable_done = false;
    TickType_t upright_since = xTaskGetTickCount();
#endif // AUTO_ENABLE

    motor_init();
    motor_standby(true);  /* Nothing moves until enabled */

#if WATCHDOG
    /* Refreshed on every loop pass (sample or stall timeout): a hung control
     * task, e.g. deadlocked on the state mutex, resets the MCU */
    board_watchdog_start(WATCHDOG_TIMEOUT_MS);
#endif // WATCHDOG

    IMU_Data_t imu_data;

    for (;;) {
#if WATCHDOG
        board_watchdog_refresh();
#endif // WATCHDOG

        if (!imu_wait_sample(&imu_data, pdMS_TO_TICKS(IMU_STALL_TIMEOUT_MS))) {
            /* No fresh attitude: acting on stale data is worse than stopping */
            robot_lock();
            if (robot.motors_enabled) {
                robot_disable();
            }
            robot_unlock();
            continue;
        }

        float acc_angle = calc_angle_from_accel(&imu_data);

        for (int f = 0; f < FILTER_COUNT; f++) {
            /* Start at the measured angle instead of converging from 0 */
            if (!filters_seeded) {
                filters[f].seed(filters[f].state, acc_angle);
            }
            angles[f] = filters[f].update(filters[f].state, imu_data.gyro_x, acc_angle, IMU_SAMPLE_RATE_S);
        }
        filters_seeded = true;

        /* 0 = vertical, positive = leaning forward */
        float tilt = angles[CONTROL_FILTER] - 90.0f;

        robot_lock();

        robot.imu = imu_data;
        robot.tilt = tilt;
        robot.is_balanced = (fabsf(tilt) < BALANCED_TILT_ANGLE);

#if AUTO_ENABLE
        /* Without a console nothing sends AT+ENABLE: start balancing once per
         * boot, after the robot has been held upright for AUTO_ENABLE_HOLD_MS */
        if (!auto_enable_done) {
            if (!robot.is_balanced) {
                upright_since = xTaskGetTickCount();
            } else if ((xTaskGetTickCount() - upright_since) >= pdMS_TO_TICKS(AUTO_ENABLE_HOLD_MS)) {
                robot_enable();
                auto_enable_done = true;
            }
        }
#endif // AUTO_ENABLE

        if (robot.motors_enabled && robot.pid_enabled) {
            robot_balance_step(tilt);
        }

#if TELEMETRY
        Telemetry_Record_t record = {
            .tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .acc_deg = acc_angle,
            .kalman = angles[FILTER_KALMAN],
            .comp = angles[FILTER_COMPLEMENTARY],
            .tilt = tilt,
            .p = robot.pid.p_term,
            .i = robot.pid.i_term,
            .d = robot.pid.d_term,
            .out = robot.pid.output,
        };
#endif // TELEMETRY

        robot_unlock();

#if TELEMETRY
        /* Never blocks, and does nothing unless AT+STREAM=1 */
        (void)telemetry_submit(&record);
#endif // TELEMETRY
    }
}

APP_MODULE(robot_module) = {
    .name = "ROBOT",
    .init = robot_init,
    .task = robot_task,
    .stack = 256,               /* filters, PID */
    .priority = APP_PRIORITY_CONTROL,
};
