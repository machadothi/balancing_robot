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
#include "board_config.h"
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

/** Balance PID gains at startup and after AT+DEFAULT. Tuned values are robot
 * properties: a board config overrides them with BOARD_DEFAULT_KP/KI/KD */
#ifdef BOARD_DEFAULT_KP
#define ROBOT_DEFAULT_KP        BOARD_DEFAULT_KP
#else
#define ROBOT_DEFAULT_KP        25.0f
#endif // BOARD_DEFAULT_KP
#ifdef BOARD_DEFAULT_KI
#define ROBOT_DEFAULT_KI        BOARD_DEFAULT_KI
#else
#define ROBOT_DEFAULT_KI        0.5f
#endif // BOARD_DEFAULT_KI
#ifdef BOARD_DEFAULT_KD
#define ROBOT_DEFAULT_KD        BOARD_DEFAULT_KD
#else
#define ROBOT_DEFAULT_KD        0.8f
#endif // BOARD_DEFAULT_KD

/** Balance output limit at startup, percent of full power (AT+OUTLIMIT) */
#ifndef BOARD_DEFAULT_OUTLIMIT
#define BOARD_DEFAULT_OUTLIMIT  100
#endif // BOARD_DEFAULT_OUTLIMIT

/** Clamp on the accumulated angle error (deg x s) */
#define PID_INTEGRAL_LIMIT      100.0f

/** Balance target at startup and after AT+DEFAULT (degrees from vertical): the
 * robot's balance point, so a board config can trim it */
#ifdef BOARD_BALANCE_SETPOINT
#define BALANCE_SETPOINT        BOARD_BALANCE_SETPOINT
#else
#define BALANCE_SETPOINT        0.0f
#endif // BOARD_BALANCE_SETPOINT

/** Beyond this tilt recovery is impossible: stop instead (degrees) */
#define MAX_TILT_ANGLE          45.0f

/** Reported as BALANCED below this tilt (degrees) */
#define BALANCED_TILT_ANGLE     5.0f

/** Smallest command that keeps each wheel turning (motor and supply property:
 * the board sets it, `pid_tune.py deadband` measures it, AT+DEADBAND changes it) */
#ifndef BOARD_MOTOR_DEADBAND_LEFT
#define BOARD_MOTOR_DEADBAND_LEFT   20
#endif // BOARD_MOTOR_DEADBAND_LEFT
#ifndef BOARD_MOTOR_DEADBAND_RIGHT
#define BOARD_MOTOR_DEADBAND_RIGHT  20
#endif // BOARD_MOTOR_DEADBAND_RIGHT

/** Drive commands (AT+VELOCITY, AT+TURN) must repeat: after this long without one,
 * speed and turn targets return to 0, so a lost phone link cannot drive the robot away */
#define DRIVE_TIMEOUT_MS        1000

/** Outer speed loop (SPEED_LOOP): every SPEED_LOOP_DIVIDER samples (100 ms) */
#define SPEED_LOOP_DIVIDER      10
#define SPEED_LOOP_PERIOD_S     (SPEED_LOOP_DIVIDER * IMU_SAMPLE_RATE_S)
/** Weight of a new speed measurement (light low-pass against encoder jitter) */
#define SPEED_FILTER_WEIGHT     0.5f
/** The outer loop moves the balance setpoint by at most this much (deg) */
#define SPEED_LOOP_MAX_TILT     4.0f
/** Clamp on the accumulated speed error (% x s) */
#define SPEED_INTEGRAL_LIMIT    100.0f

/** Wheel speed at full power, encoder counts/s: AT+VELOCITY is a percentage of it */
#ifndef BOARD_WHEEL_MAX_CPS
#define BOARD_WHEEL_MAX_CPS     4500.0f
#endif // BOARD_WHEEL_MAX_CPS
#ifndef BOARD_SPEED_KP
#define BOARD_SPEED_KP          0.05f   /**< deg of lean per % of speed error */
#endif // BOARD_SPEED_KP
#ifndef BOARD_SPEED_KI
#define BOARD_SPEED_KI          0.02f   /**< deg per (% x s) */
#endif // BOARD_SPEED_KI
/** Balance D term from the gyro rate instead of the angle difference (AT+DGYRO) */
#ifndef BOARD_D_FROM_GYRO
#define BOARD_D_FROM_GYRO       0
#endif // BOARD_D_FROM_GYRO

/** Speed loop on at start-up (AT+VLOOP changes it) */
#ifndef BOARD_SPEED_LOOP_DEFAULT
#define BOARD_SPEED_LOOP_DEFAULT 0
#endif // BOARD_SPEED_LOOP_DEFAULT

/** Gyro weight of the complementary filter: a robot whose wheels accelerate
 * hard needs more, since the accelerometer then reads that acceleration as tilt */
#ifndef BOARD_COMPLEMENTARY_ALPHA
#define BOARD_COMPLEMENTARY_ALPHA   COMPLEMENTARY_ALPHA
#endif // BOARD_COMPLEMENTARY_ALPHA

/* ==========================================================================
 * Shared State
 * ========================================================================== */

Robot_t robot = {
    .pid = {
        .kp = ROBOT_DEFAULT_KP,
        .ki = ROBOT_DEFAULT_KI,
        .kd = ROBOT_DEFAULT_KD,
        .integral_limit = PID_INTEGRAL_LIMIT,
        .output_limit = BOARD_DEFAULT_OUTLIMIT * MOTOR_COMMAND_MAX / 100.0f,
    },
    .pid_enabled = true,
    .setpoint = BALANCE_SETPOINT,
    .deadband_left = BOARD_MOTOR_DEADBAND_LEFT,
    .deadband_right = BOARD_MOTOR_DEADBAND_RIGHT,
    .comp_alpha = BOARD_COMPLEMENTARY_ALPHA,
    .speed_loop = BOARD_SPEED_LOOP_DEFAULT,
    .d_from_gyro = BOARD_D_FROM_GYRO,
    .speed_pid = {
        .kp = BOARD_SPEED_KP,
        .ki = BOARD_SPEED_KI,
        .integral_limit = SPEED_INTEGRAL_LIMIT,
        .output_limit = SPEED_LOOP_MAX_TILT,
    },
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
    pid_reset(&robot.speed_pid);
    robot.speed_offset = 0.0f;
    robot.motors_enabled = true;
    motor_standby(false);
}

void robot_disable(void) {
    robot.armed = false;        /* a stop or a fall also cancels arming */
    robot.motors_enabled = false;
    motor_set(MOTOR_LEFT, 0);
    motor_set(MOTOR_RIGHT, 0);
    motor_standby(true);
    pid_reset(&robot.pid);
}

void robot_toggle_armed(void) {
    robot_lock();
    if (robot.armed || robot.motors_enabled) {
        robot_disable();
    } else {
        robot.armed = true;
    }
    robot_unlock();
}

void robot_restore_defaults(void) {
    robot.pid.kp = ROBOT_DEFAULT_KP;
    robot.pid.ki = ROBOT_DEFAULT_KI;
    robot.pid.kd = ROBOT_DEFAULT_KD;
    robot.target_velocity = 0.0f;
    robot.turn_rate = 0.0f;
    robot.setpoint = BALANCE_SETPOINT;
}

/* ==========================================================================
 * Control
 * ========================================================================== */

/** One balance update: safety cut-off, PID, mixing, motors; tilt_rate from the gyro (deg/s) */
static void robot_balance_step(float tilt, float tilt_rate) {
    if (fabsf(tilt) > MAX_TILT_ANGLE) {
        robot_disable();
        return;
    }

    /* Lean forward (tilt > 0) -> drive forward (output > 0): the wheels move
     * under the falling body. error = setpoint - tilt would push them away. */
    /* The speed loop leans the robot back while it rolls forward too fast */
    float setpoint = robot.setpoint - robot.speed_offset;
    float output = robot.d_from_gyro
        ? pid_update_rate(&robot.pid, tilt - setpoint, tilt_rate, IMU_SAMPLE_RATE_S)
        : pid_update(&robot.pid, tilt - setpoint, IMU_SAMPLE_RATE_S);
    /* The PID's output limit (AT+OUTLIMIT) also caps each wheel after mixing */
    Mixer_Output_t wheels = mixer_mix(output, robot.turn_rate, (int16_t)robot.pid.output_limit,
                                     robot.deadband_left, robot.deadband_right);

    motor_set(MOTOR_LEFT, wheels.left);
    motor_set(MOTOR_RIGHT, wheels.right);
}

#if SPEED_LOOP
/**
 * @brief Every SPEED_LOOP_DIVIDER samples: measure the forward speed and run
 *        the outer loop (caller holds the lock)
 *
 * The average of both wheels is the forward speed; turning cancels out. To
 * stop rolling forward, the robot must lean back, so a positive speed error
 * gives a positive offset that is subtracted from the balance setpoint.
 */
static void robot_speed_step(void) {
    static uint8_t divider;
    static bool started;
    static int32_t previous;

    if (++divider < SPEED_LOOP_DIVIDER) {
        return;
    }
    divider = 0;

    int32_t now = motor_get_encoder(MOTOR_LEFT) + motor_get_encoder(MOTOR_RIGHT);
    if (started) {
        float cps = (float)(now - previous) / 2.0f / SPEED_LOOP_PERIOD_S;
        float percent = 100.0f * cps / BOARD_WHEEL_MAX_CPS;
        robot.speed += SPEED_FILTER_WEIGHT * (percent - robot.speed);
    }
    previous = now;
    started = true;

    if (robot.speed_loop && robot.motors_enabled && robot.pid_enabled) {
        robot.speed_offset = pid_update(&robot.speed_pid, robot.speed - robot.target_velocity,
                                        SPEED_LOOP_PERIOD_S);
    } else {
        pid_reset(&robot.speed_pid);
        robot.speed_offset = 0.0f;
    }
}
#endif // SPEED_LOOP

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
    complementary_init_custom(&complementary, robot.comp_alpha);

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

        /* The board config maps the sensor axes onto the balance plane */
        IMU_Tilt_t measured = imu_tilt(&imu_data);
        float acc_angle = measured.acc_deg;

        for (int f = 0; f < FILTER_COUNT; f++) {
            /* Start at the measured angle instead of converging from 0 */
            if (!filters_seeded) {
                filters[f].seed(filters[f].state, acc_angle);
            }
            angles[f] = filters[f].update(filters[f].state, measured.rate_dps, acc_angle, IMU_SAMPLE_RATE_S);
        }
        filters_seeded = true;

        /* 0 = upright, positive = leaning forward */
        float tilt = angles[CONTROL_FILTER];

        robot_lock();

        /* AT+ALPHA takes effect on the next sample */
        complementary.alpha = robot.comp_alpha;

        robot.imu = imu_data;
        robot.tilt = tilt;
        robot.is_balanced = (fabsf(tilt) < BALANCED_TILT_ANGLE);

        /* Armed (button): lifted upright, so start balancing */
        if (robot.armed && robot.is_balanced) {
            robot_enable();
            robot.armed = false;
        }

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

        if ((robot.target_velocity != 0.0f || robot.turn_rate != 0.0f) &&
            (xTaskGetTickCount() - robot.drive_tick) > pdMS_TO_TICKS(DRIVE_TIMEOUT_MS)) {
            robot.target_velocity = 0.0f;   /* dead-man: the driver went quiet */
            robot.turn_rate = 0.0f;
        }

#if SPEED_LOOP
        robot_speed_step();
#endif // SPEED_LOOP

        if (robot.motors_enabled && robot.pid_enabled) {
            robot_balance_step(tilt, measured.rate_dps);
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
            .speed = robot.speed,
            .setpoint = robot.setpoint - robot.speed_offset,
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
