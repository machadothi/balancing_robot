/**
 * @file robot.h
 * @brief Robot control task interface
 * 
 * Main control loop for the self-balancing robot.
 * Reads IMU data, applies sensor fusion filters, and controls motors.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Robot control task
 * 
 * FreeRTOS task that implements the main control loop:
 * 1. Receives IMU data from queue
 * 2. Calculates tilt angle from accelerometer
 * 3. Applies Kalman and Complementary filters
 * 4. Outputs debug data via UART
 * 
 * @param args Task arguments (unused)
 */
void robot_task(void *args);

/**
 * @brief Arm or stop balancing, e.g. from a button
 *
 * Off -> armed: balancing starts by itself once the robot is upright
 * (the BALANCED threshold). Armed or balancing -> off: motors stopped and
 * the driver in standby, like AT+DISABLE.
 */
void robot_toggle_armed(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ROBOT_H
