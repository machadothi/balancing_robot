/**
 * @file robot.c
 * @brief Robot control task implementation
 * 
 * Implements the main control loop for the self-balancing robot.
 * Integrates with AT command interface for remote control.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "config.h"
#include "board/board.h"
#include "robot/robot.h"
#include "imu/imu.h"
#include "filter/filter.h"
#include "cmd/at_cmd.h"
#if TELEMETRY
#include "telemetry/telemetry.h"
#endif // TELEMETRY
#include "log/log.h"
#include "motor/motor.h"

/* ==========================================================================
 * Constants
 * ========================================================================== */

#define RAD_TO_DEG  (180.0f / 3.14159265f)

/** Balance PID gains at startup and after AT+DEFAULT */
#define ROBOT_DEFAULT_KP    25.0f
#define ROBOT_DEFAULT_KI    0.5f
#define ROBOT_DEFAULT_KD    0.8f

/** Setpoint angle for balance (degrees from vertical) */
#define BALANCE_SETPOINT    0.0f

/** Maximum allowed tilt before disabling motors (degrees) */
#define MAX_TILT_ANGLE      45.0f

/** Minimum motor PWM to overcome static friction */
#define MOTOR_DEADBAND      20

/** Maximum motor PWM output */
#define MOTOR_MAX_PWM       255

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Robot state for AT command interface */
static AT_RobotState_t robot_state = {
    .acc_x = 0.0f,
    .acc_y = 0.0f,
    .acc_z = 0.0f,
    .gyro_x = 0.0f,
    .gyro_y = 0.0f,
    .gyro_z = 0.0f,
    .angle = 0.0f,
    .velocity = 0.0f,
    .target_velocity = 0.0f,
    .turn_rate = 0.0f,
    .kp = ROBOT_DEFAULT_KP,
    .ki = ROBOT_DEFAULT_KI,
    .kd = ROBOT_DEFAULT_KD,
    .motors_enabled = false,
    .pid_enabled = true,  /* PID enabled by default */
    .is_balanced = false,
};

/** Guards robot_state and motor commands against the AT handlers (UART RX task) */
static SemaphoreHandle_t state_mutex = NULL;

#if TELEMETRY
/** AT+STREAM: submit a telemetry record every sample */
static bool stream_enabled = false;
#endif // TELEMETRY

/* ==========================================================================
 * PID Controller State
 * ========================================================================== */

typedef struct {
    float integral;         /**< Accumulated integral term */
    float prev_error;       /**< Previous error for derivative */
    float integral_limit;   /**< Anti-windup limit */
    float output;           /**< Last PID output */
    float p_term;           /**< Last terms, for telemetry */
    float i_term;
    float d_term;
} PID_State_t;

static PID_State_t pid = {
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 100.0f,  /* Limit integral windup */
    .output = 0.0f,
};

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Calculate tilt angle from accelerometer data
 * 
 * Uses atan2 for proper quadrant handling.
 * IMU is mounted with X-axis vertical (ax ≈ -1g when level).
 * 
 * @param data Pointer to IMU data structure
 * @return Tilt angle in degrees
 */
static float calc_angle_from_accel(const IMU_Data_t *data) {
    return atan2f(data->acc_y, -data->acc_x) * RAD_TO_DEG;
}

#if CONSOLE_ANY
/**
 * @brief Convert speed percentage to PWM value and direction
 * 
 * @param speed     Speed percentage (-100 to 100)
 * @param pwm       Output PWM value (0-255)
 * @param forward   Output direction (true = forward)
 */
static void speed_to_pwm(float speed, uint8_t *pwm, bool *forward) {
    if (speed >= 0.0f) {
        *forward = true;
        *pwm = (uint8_t)(speed * 2.55f);  /* 100% -> 255 */
    } else {
        *forward = false;
        *pwm = (uint8_t)(-speed * 2.55f);
    }
}
#endif // CONSOLE_ANY

/**
 * @brief Reset PID controller state
 * 
 * Clears integral accumulator and previous error.
 * Call when enabling motors or after a fall.
 */
static void pid_reset(void) {
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.output = 0.0f;
    pid.p_term = 0.0f;
    pid.i_term = 0.0f;
    pid.d_term = 0.0f;
}

static void robot_lock(void) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
}

static void robot_unlock(void) {
    xSemaphoreGive(state_mutex);
}

/**
 * @brief Calculate PID output for balance control
 * 
 * @param angle     Current tilt angle (degrees, 0 = vertical)
 * @param dt        Time delta in seconds
 * @return          Motor control output (-255 to 255)
 */
static float pid_compute(float angle, float dt) {
    /* Error = setpoint - current angle */
    float error = BALANCE_SETPOINT - angle;
    
    /* Proportional term */
    float p_term = robot_state.kp * error;
    
    /* Integral term with anti-windup */
    pid.integral += error * dt;
    if (pid.integral > pid.integral_limit) {
        pid.integral = pid.integral_limit;
    } else if (pid.integral < -pid.integral_limit) {
        pid.integral = -pid.integral_limit;
    }
    float i_term = robot_state.ki * pid.integral;
    
    /* Derivative term (on error) */
    float derivative = (error - pid.prev_error) / dt;
    float d_term = robot_state.kd * derivative;
    pid.prev_error = error;
    
    /* Sum all terms */
    pid.p_term = p_term;
    pid.i_term = i_term;
    pid.d_term = d_term;
    pid.output = p_term + i_term + d_term;
    
    /* Clamp output to motor range */
    if (pid.output > MOTOR_MAX_PWM) {
        pid.output = MOTOR_MAX_PWM;
    } else if (pid.output < -MOTOR_MAX_PWM) {
        pid.output = -MOTOR_MAX_PWM;
    }
    
    return pid.output;
}

/**
 * @brief Apply motor control with deadband and differential steering
 * 
 * @param output    PID output (-255 to 255)
 */
static void apply_motor_control(float output) {
    /* Add turn rate for differential steering */
    float left_output = output + robot_state.turn_rate;
    float right_output = output - robot_state.turn_rate;
    
    /* Determine direction and magnitude */
    bool left_forward = (left_output >= 0.0f);
    bool right_forward = (right_output >= 0.0f);
    
    /* Turn rate can push |output| past 255: saturate before narrowing to uint8_t */
    uint8_t left_pwm = (uint8_t)fminf(fabsf(left_output), (float)MOTOR_MAX_PWM);
    uint8_t right_pwm = (uint8_t)fminf(fabsf(right_output), (float)MOTOR_MAX_PWM);
    
    /* Apply deadband compensation */
    if (left_pwm > 0 && left_pwm < MOTOR_DEADBAND) {
        left_pwm = MOTOR_DEADBAND;
    }
    if (right_pwm > 0 && right_pwm < MOTOR_DEADBAND) {
        right_pwm = MOTOR_DEADBAND;
    }
    
    /* Set motor direction and speed */
    motor1_set_direction(left_forward);
    motor2_set_direction(right_forward);
    motor1_set_speed(left_pwm);
    motor2_set_speed(right_pwm);
}

#if CONSOLE_ANY
/**
 * @brief AT command set callback
 * 
 * Handles SET commands from AT interface.
 * @param param     Parameter name
 * @param value     First value
 * @param value2    Second value (for dual-parameter commands like SPEED)
 */
static bool at_set_handler(const char *param, float value, float value2) {
#if TELEMETRY
    if (strcmp(param, "STREAM") == 0) {
        stream_enabled = (value != 0.0f);
        return true;
    }
#endif // TELEMETRY
    if (strcmp(param, "SPEED") == 0) {
        /* Direct wheel speed control */
        uint8_t pwm_left, pwm_right;
        bool dir_left, dir_right;
        
        speed_to_pwm(value, &pwm_left, &dir_left);
        speed_to_pwm(value2, &pwm_right, &dir_right);
        
        motor1_set_direction(dir_left);
        motor2_set_direction(dir_right);
        motor1_set_speed(pwm_left);
        motor2_set_speed(pwm_right);
        
        /* Update state */
        robot_state.speed_left = value;
        robot_state.speed_right = value2;
        return true;
    }
    else if (strcmp(param, "VELOCITY") == 0) {
        robot_state.target_velocity = value;
        return true;
    }
    else if (strcmp(param, "TURN") == 0) {
        robot_state.turn_rate = value;
        return true;
    }
    else if (strcmp(param, "KP") == 0) {
        robot_state.kp = value;
        return true;
    }
    else if (strcmp(param, "KI") == 0) {
        robot_state.ki = value;
        return true;
    }
    else if (strcmp(param, "KD") == 0) {
        robot_state.kd = value;
        return true;
    }
    return false;
}

/**
 * @brief AT command execute callback
 * 
 * Handles EXECUTE commands from AT interface.
 */
static AT_Result_t at_exec_handler(const char *cmd) {
    if (strcmp(cmd, "ENABLE") == 0) {
        pid_reset();  /* Clear PID state before enabling */
        robot_state.motors_enabled = true;
        motor_standby(false);  /* Exit standby */
        return AT_OK;
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        robot_state.motors_enabled = false;
        motor_standby(true);  /* Enter standby */
        return AT_OK;
    }
#if AT_CMD_PID_TOGGLE
    else if (strcmp(cmd, "PID") == 0) {
        /* Toggle PID controller */
        robot_state.pid_enabled = !robot_state.pid_enabled;
        if (!robot_state.pid_enabled) {
            pid_reset();
        }
        return AT_OK;
    }
    else if (strcmp(cmd, "PIDON") == 0) {
        robot_state.pid_enabled = true;
        return AT_OK;
    }
    else if (strcmp(cmd, "PIDOFF") == 0) {
        robot_state.pid_enabled = false;
        pid_reset();
        /* Stop motors when disabling PID */
        motor1_set_speed(0);
        motor2_set_speed(0);
        return AT_OK;
    }
#endif // AT_CMD_PID_TOGGLE
    else if (strcmp(cmd, "STOP") == 0) {
        robot_state.motors_enabled = false;
        robot_state.target_velocity = 0.0f;
        robot_state.turn_rate = 0.0f;
        robot_state.speed_left = 0.0f;
        robot_state.speed_right = 0.0f;
        motor1_set_speed(0);
        motor2_set_speed(0);
        motor_standby(true);
        return AT_OK;
    }
    else if (strcmp(cmd, "SAVE") == 0 || strcmp(cmd, "LOAD") == 0) {
        /* Parameter storage in flash is not implemented yet */
        return AT_ERROR;
    }
    else if (strcmp(cmd, "DEFAULT") == 0) {
        robot_state.kp = ROBOT_DEFAULT_KP;
        robot_state.ki = ROBOT_DEFAULT_KI;
        robot_state.kd = ROBOT_DEFAULT_KD;
        robot_state.target_velocity = 0.0f;
        robot_state.turn_rate = 0.0f;
        return AT_OK;
    }
    return AT_ERROR_UNKNOWN_CMD;
}
#endif // CONSOLE_ANY

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void robot_task(void *args) {
    (void)args;
    
    IMU_Data_t imu_data;
    
    /* Initialize filters */
    KalmanFilter_t kalman;
    ComplementaryFilter_t complementary;
    
    kalman_init(&kalman);
    complementary_init(&complementary);
    bool filters_seeded = false;

#if AUTO_ENABLE
    bool auto_enable_done = false;
    TickType_t upright_since = xTaskGetTickCount();
#endif // AUTO_ENABLE

    state_mutex = xSemaphoreCreateMutex();

    /* Initialize motor driver */
    motor_init();
    motor_standby(true);  /* Start in standby */

#if CONSOLE_ANY
    /* Register AT command handlers */
    at_cmd_set_lock(robot_lock, robot_unlock);
    at_cmd_set_state(&robot_state);
    at_cmd_set_callback(at_set_handler);
    at_cmd_exec_callback(at_exec_handler);
#endif // CONSOLE_ANY

#if WATCHDOG
    /* Refreshed on every loop pass (sample or stall timeout): a hung control
     * task, e.g. deadlocked on the state mutex, resets the MCU */
    board_watchdog_start(WATCHDOG_TIMEOUT_MS);
#endif // WATCHDOG

    for (;;) {
#if WATCHDOG
        board_watchdog_refresh();
#endif // WATCHDOG

        if (xQueueReceive(imu_content, &imu_data, pdMS_TO_TICKS(IMU_STALL_TIMEOUT_MS)) != pdPASS) {
            /* No fresh attitude: acting on stale data is worse than stopping */
            robot_lock();
            if (robot_state.motors_enabled) {
                robot_state.motors_enabled = false;
                motor1_set_speed(0);
                motor2_set_speed(0);
                motor_standby(true);
                pid_reset();
            }
            robot_unlock();
            continue;
        }

        /* Calculate angle from accelerometer */
        float acc_angle = calc_angle_from_accel(&imu_data);

        /* Start both filters at the measured angle instead of converging from 0 */
        if (!filters_seeded) {
            kalman.angle = acc_angle;
            complementary.angle = acc_angle;
            filters_seeded = true;
        }

        /* The selected filter drives the controller; with telemetry both run so
         * AT+STREAM can compare them */
#if TELEMETRY || ATTITUDE_FILTER_KALMAN
        float kalman_angle = kalman_update(&kalman, imu_data.gyro_x,
            acc_angle, IMU_SAMPLE_RATE_S);
#endif // TELEMETRY || ATTITUDE_FILTER_KALMAN
#if TELEMETRY || ATTITUDE_FILTER_COMPLEMENTARY
        float comp_angle = complementary_update(&complementary, imu_data.gyro_x,
            acc_angle, IMU_SAMPLE_RATE_S);
#endif // TELEMETRY || ATTITUDE_FILTER_COMPLEMENTARY

#if ATTITUDE_FILTER_KALMAN
        float filtered_angle = kalman_angle;
#else
        float filtered_angle = comp_angle;
#endif // ATTITUDE_FILTER_KALMAN

        /* Normalize angle: 0 = vertical, positive = tilting forward */
        float tilt_angle = filtered_angle - 90.0f;

        robot_lock();

        /* Update robot state with IMU data */
        robot_state.acc_x = imu_data.acc_x;
        robot_state.acc_y = imu_data.acc_y;
        robot_state.acc_z = imu_data.acc_z;
        robot_state.gyro_x = imu_data.gyro_x;
        robot_state.gyro_y = imu_data.gyro_y;
        robot_state.gyro_z = imu_data.gyro_z;
        robot_state.angle = tilt_angle;

        /* Check if balanced (within ~5 degrees of vertical) */
        robot_state.is_balanced = (fabsf(tilt_angle) < 5.0f);

#if AUTO_ENABLE
        /* Without a console nothing sends AT+ENABLE: start balancing once per
         * boot, after the robot has been held upright for AUTO_ENABLE_HOLD_MS */
        if (!auto_enable_done) {
            if (!robot_state.is_balanced) {
                upright_since = xTaskGetTickCount();
            } else if ((xTaskGetTickCount() - upright_since) >= pdMS_TO_TICKS(AUTO_ENABLE_HOLD_MS)) {
                pid_reset();
                robot_state.motors_enabled = true;
                motor_standby(false);
                auto_enable_done = true;
            }
        }
#endif // AUTO_ENABLE

        /* PID balance control */
        if (robot_state.motors_enabled && robot_state.pid_enabled) {
            /* Safety: disable if tilted too far */
            if (fabsf(tilt_angle) > MAX_TILT_ANGLE) {
                robot_state.motors_enabled = false;
                motor1_set_speed(0);
                motor2_set_speed(0);
                motor_standby(true);
                pid_reset();
            } else {
                /* Compute PID and apply to motors */
                float output = pid_compute(tilt_angle, IMU_SAMPLE_RATE_S);
                apply_motor_control(output);
            }
        }

#if TELEMETRY
        bool stream = stream_enabled;
        Telemetry_Record_t record = {
            .tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            .acc_deg = acc_angle,
            .kalman = kalman_angle,
            .comp = comp_angle,
            .tilt = tilt_angle,
            .p = pid.p_term,
            .i = pid.i_term,
            .d = pid.d_term,
            .out = pid.output,
        };
#endif // TELEMETRY
        robot_unlock();

#if TELEMETRY
        /* Never blocks: formatting and UART output happen in telemetry_task */
        if (stream) {
            (void)telemetry_submit(&record);
        }
#endif // TELEMETRY
    }
}