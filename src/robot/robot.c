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

#include "config.h"
#include "robot/robot.h"
#include "imu/imu.h"
#include "filter/filter.h"
#include "drivers/uart.h"
#include "cmd/at_cmd.h"
#include "log/log.h"
#include "motor/motor.h"

/* ==========================================================================
 * Constants
 * ========================================================================== */

#define RAD_TO_DEG  (180.0f / 3.14159265f)

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
    .kp = 10.0f,
    .ki = 0.5f,
    .kd = 1.0f,
    .motors_enabled = false,
    .is_balanced = false,
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

/**
 * @brief AT command set callback
 * 
 * Handles SET commands from AT interface.
 * @param param     Parameter name
 * @param value     First value
 * @param value2    Second value (for dual-parameter commands like SPEED)
 */
static bool at_set_handler(const char *param, float value, float value2) {
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
static bool at_exec_handler(const char *cmd) {
    if (strcmp(cmd, "ENABLE") == 0) {
        robot_state.motors_enabled = true;
        motor_standby(false);  /* Exit standby */
        return true;
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        robot_state.motors_enabled = false;
        motor_standby(true);  /* Enter standby */
        return true;
    }
    else if (strcmp(cmd, "STOP") == 0) {
        robot_state.motors_enabled = false;
        robot_state.target_velocity = 0.0f;
        robot_state.turn_rate = 0.0f;
        motor1_set_speed(0);
        motor2_set_speed(0);
        motor_standby(true);
        return true;
    }
    else if (strcmp(cmd, "SAVE") == 0) {
        /* TODO: Save to flash */
        return true;
    }
    else if (strcmp(cmd, "LOAD") == 0) {
        /* TODO: Load from flash */
        return true;
    }
    else if (strcmp(cmd, "DEFAULT") == 0) {
        robot_state.kp = 10.0f;
        robot_state.ki = 0.5f;
        robot_state.kd = 1.0f;
        robot_state.target_velocity = 0.0f;
        robot_state.turn_rate = 0.0f;
        return true;
    }
    return false;
}

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
    
    /* Initialize motor driver */
    motor_init();
    motor_standby(true);  /* Start in standby */
    
    /* Register AT command handlers */
    at_cmd_set_state(&robot_state);
    at_cmd_set_callback(at_set_handler);
    at_cmd_exec_callback(at_exec_handler);

    for (;;) {
        if (xQueueReceive(imu_content, &imu_data, 500) == pdPASS) {
            /* Update robot state with IMU data */
            robot_state.acc_x = imu_data.acc_x;
            robot_state.acc_y = imu_data.acc_y;
            robot_state.acc_z = imu_data.acc_z;
            robot_state.gyro_x = imu_data.gyro_x;
            robot_state.gyro_y = imu_data.gyro_y;
            robot_state.gyro_z = imu_data.gyro_z;
            
            /* Calculate angle from accelerometer */
            float acc_angle = calc_angle_from_accel(&imu_data);
            
            /* Update both filters */
            float kalman_angle = kalman_update(&kalman, imu_data.gyro_x, 
                acc_angle, IMU_SAMPLE_RATE_S);
            float comp_angle = complementary_update(&complementary, imu_data.gyro_x, 
                acc_angle, IMU_SAMPLE_RATE_S);
            
            /* Store filtered angle (use complementary for now) */
            robot_state.angle = comp_angle;
            
            /* Check if balanced (within ~5 degrees of vertical) */
            robot_state.is_balanced = (fabsf(comp_angle - 90.0f) < 5.0f);

            /* Suppress unused variable warnings */
            (void)kalman_angle;
        } else {
            taskYIELD();
        }
    }
}