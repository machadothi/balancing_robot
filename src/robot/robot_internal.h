/**
 * @file robot_internal.h
 * @brief State shared by the robot control loop and its AT commands
 *
 * Private to src/robot/. Every access to `robot` goes through robot_lock():
 * the control task and the UART RX task (AT commands) both use it.
 */

#ifndef ROBOT_INTERNAL_H
#define ROBOT_INTERNAL_H

#include <stdbool.h>

#include "config.h"
#include "control/pid.h"
#include "imu/imu.h"

typedef struct {
    IMU_Data_t imu;             /**< Latest sample, physical units */
    float tilt;                 /**< Filtered tilt, degrees, 0 = upright */
    float target_velocity;      /**< AT+VELOCITY / AT+TARGET, not used by the control law yet */
    float turn_rate;            /**< Differential term added to the wheels */
    float speed_left;           /**< Last AT+SPEED values, percent */
    float speed_right;
    bool motors_enabled;
    bool pid_enabled;
    bool is_balanced;           /**< |tilt| below the "balanced" threshold */
    PID_t pid;                  /**< Balance controller, gains included */
} Robot_t;

extern Robot_t robot;

void robot_lock(void);
void robot_unlock(void);

/* The functions below expect the caller to hold the lock */

/** Reset the PID and leave standby: the loop starts balancing (if the PID is on) */
void robot_enable(void);

/** Stop balancing, zero both wheels, enter standby, reset the PID */
void robot_disable(void);

/** Default gains, no turn, no target velocity */
void robot_restore_defaults(void);

#if CONSOLE_ANY
/** Register the robot's AT commands (robot_commands.c) */
void robot_commands_register(void);
#endif // CONSOLE_ANY

#endif // ROBOT_INTERNAL_H
