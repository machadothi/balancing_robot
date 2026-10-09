/**
 * @file robot_commands.c
 * @brief AT commands of the balance controller
 *
 * Every handler takes the robot lock for as short as possible: queries copy
 * the value under the lock and format it afterwards.
 */

#include <float.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "cmd/at_cmd.h"
#include "motor/motor.h"
#include "robot/robot_internal.h"
#include "util/fmt.h"

/* ==========================================================================
 * Queries
 * ========================================================================== */

#define DEFINE_FLOAT_QUERY(fn, expression, decimals)        \
    static AT_Result_t fn(char *value, size_t size) {       \
        robot_lock();                                       \
        float v = (expression);                             \
        robot_unlock();                                     \
        fmt_fixed(value, size, v, (decimals));              \
        return AT_OK;                                       \
    }

DEFINE_FLOAT_QUERY(query_acc_x, robot.imu.acc_x, 3)
DEFINE_FLOAT_QUERY(query_acc_y, robot.imu.acc_y, 3)
DEFINE_FLOAT_QUERY(query_acc_z, robot.imu.acc_z, 3)
DEFINE_FLOAT_QUERY(query_gyro_x, robot.imu.gyro_x, 3)
DEFINE_FLOAT_QUERY(query_gyro_y, robot.imu.gyro_y, 3)
DEFINE_FLOAT_QUERY(query_gyro_z, robot.imu.gyro_z, 3)
DEFINE_FLOAT_QUERY(query_angle, robot.tilt, 2)
DEFINE_FLOAT_QUERY(query_target, robot.target_velocity, 2)
DEFINE_FLOAT_QUERY(query_turn, robot.turn_rate, 2)
DEFINE_FLOAT_QUERY(query_kp, robot.pid.kp, 4)
DEFINE_FLOAT_QUERY(query_ki, robot.pid.ki, 4)
DEFINE_FLOAT_QUERY(query_kd, robot.pid.kd, 4)
DEFINE_FLOAT_QUERY(query_setpoint, robot.setpoint, 2)
DEFINE_FLOAT_QUERY(query_alpha, robot.comp_alpha, 3)

#if SPEED_LOOP
DEFINE_FLOAT_QUERY(query_velocity, robot.speed, 2)
DEFINE_FLOAT_QUERY(query_vkp, robot.speed_pid.kp, 4)
DEFINE_FLOAT_QUERY(query_vki, robot.speed_pid.ki, 4)

static AT_Result_t query_vloop(char *value, size_t size) {
    robot_lock();
    bool on = robot.speed_loop;
    robot_unlock();
    snprintf(value, size, "%d", on ? 1 : 0);
    return AT_OK;
}
#else
static AT_Result_t query_velocity(char *value, size_t size) {
    /* No speed measurement without quadrature encoders */
    snprintf(value, size, "0.00");
    return AT_OK;
}
#endif // SPEED_LOOP

static AT_Result_t query_status(char *value, size_t size) {
    robot_lock();
    bool enabled = robot.motors_enabled;
    bool balanced = robot.is_balanced;
    robot_unlock();

    snprintf(value, size, "%s,%s", enabled ? "ENABLED" : "DISABLED",
             balanced ? "BALANCED" : "UNBALANCED");
    return AT_OK;
}

static AT_Result_t query_speed(char *value, size_t size) {
    robot_lock();
    float left = robot.speed_left;
    float right = robot.speed_right;
    robot_unlock();

    char l[16], r[16];
    snprintf(value, size, "%s,%s", fmt_fixed(l, sizeof(l), left, 1), fmt_fixed(r, sizeof(r), right, 1));
    return AT_OK;
}

static AT_Result_t query_deadband(char *value, size_t size) {
    robot_lock();
    int left = robot.deadband_left;
    int right = robot.deadband_right;
    robot_unlock();

    snprintf(value, size, "%d,%d", left, right);
    return AT_OK;
}

static AT_Result_t query_outlimit(char *value, size_t size) {
    robot_lock();
    float limit = robot.pid.output_limit;
    robot_unlock();

    snprintf(value, size, "%d", (int)(limit * 100.0f / MOTOR_COMMAND_MAX + 0.5f));
    return AT_OK;
}

/** Everything a remote display needs, in one short reply (Bluetooth is 9600 baud):
 * enabled,balanced,tilt,speed,setpoint,output,encoder_left,encoder_right */
static AT_Result_t query_live(char *value, size_t size) {
    robot_lock();
    bool enabled = robot.motors_enabled;
    bool balanced = robot.is_balanced;
    float tilt = robot.tilt;
    float speed = robot.speed;
    float setpoint = robot.setpoint - robot.speed_offset;
    float output = robot.pid.output;
    robot_unlock();
    int32_t left = motor_get_encoder(MOTOR_LEFT);
    int32_t right = motor_get_encoder(MOTOR_RIGHT);

    char t[16], v[16], s[16];
    snprintf(value, size, "%d,%d,%s,%s,%s,%d,%ld,%ld", enabled ? 1 : 0, balanced ? 1 : 0,
             fmt_fixed(t, sizeof(t), tilt, 2), fmt_fixed(v, sizeof(v), speed, 1),
             fmt_fixed(s, sizeof(s), setpoint, 2), (int)output, (long)left, (long)right);
    return AT_OK;
}

static AT_Result_t query_encoders(char *value, size_t size) {
    int32_t left = motor_get_encoder(MOTOR_LEFT);
    int32_t right = motor_get_encoder(MOTOR_RIGHT);
    snprintf(value, size, "%ld,%ld", (long)left, (long)right);
    return AT_OK;
}

#if AT_CMD_ALL_QUERY
static AT_Result_t query_all(char *value, size_t size) {
    robot_lock();
    IMU_Data_t imu = robot.imu;
    float tilt = robot.tilt;
    robot_unlock();

    char f[7][16];
    snprintf(value, size, "%s,%s,%s,%s,%s,%s,%s",
             fmt_fixed(f[0], sizeof(f[0]), imu.acc_x, 3), fmt_fixed(f[1], sizeof(f[1]), imu.acc_y, 3),
             fmt_fixed(f[2], sizeof(f[2]), imu.acc_z, 3), fmt_fixed(f[3], sizeof(f[3]), imu.gyro_x, 3),
             fmt_fixed(f[4], sizeof(f[4]), imu.gyro_y, 3), fmt_fixed(f[5], sizeof(f[5]), imu.gyro_z, 3),
             fmt_fixed(f[6], sizeof(f[6]), tilt, 2));
    return AT_OK;
}
#endif // AT_CMD_ALL_QUERY

/* ==========================================================================
 * Set commands (values arrive range-checked)
 * ========================================================================== */

#define DEFINE_FLOAT_SET(fn, field)                         \
    static AT_Result_t fn(const float *values) {            \
        robot_lock();                                       \
        (field) = values[0];                                \
        robot_unlock();                                     \
        return AT_OK;                                       \
    }

/* Drive targets: each one restarts the dead-man timeout (DRIVE_TIMEOUT_MS) */
static AT_Result_t set_target(const float *values) {
    robot_lock();
    robot.target_velocity = values[0];
    robot.drive_tick = (uint32_t)xTaskGetTickCount();
    robot_unlock();
    return AT_OK;
}

static AT_Result_t set_turn(const float *values) {
    robot_lock();
    robot.turn_rate = values[0];
    robot.drive_tick = (uint32_t)xTaskGetTickCount();
    robot_unlock();
    return AT_OK;
}
DEFINE_FLOAT_SET(set_kp, robot.pid.kp)
DEFINE_FLOAT_SET(set_ki, robot.pid.ki)
DEFINE_FLOAT_SET(set_kd, robot.pid.kd)
DEFINE_FLOAT_SET(set_setpoint, robot.setpoint)
DEFINE_FLOAT_SET(set_alpha, robot.comp_alpha)
static AT_Result_t query_dgyro(char *value, size_t size) {
    robot_lock();
    bool on = robot.d_from_gyro;
    robot_unlock();
    snprintf(value, size, "%d", on ? 1 : 0);
    return AT_OK;
}

static AT_Result_t set_dgyro(const float *values) {
    robot_lock();
    robot.d_from_gyro = (values[0] != 0.0f);
    robot_unlock();
    return AT_OK;
}

#if SPEED_LOOP
DEFINE_FLOAT_SET(set_vkp, robot.speed_pid.kp)
DEFINE_FLOAT_SET(set_vki, robot.speed_pid.ki)

static AT_Result_t set_vloop(const float *values) {
    robot_lock();
    robot.speed_loop = (values[0] != 0.0f);
    pid_reset(&robot.speed_pid);
    robot_unlock();
    return AT_OK;
}
#endif // SPEED_LOOP

static AT_Result_t set_deadband(const float *values) {
    robot_lock();
    robot.deadband_left = (int16_t)values[0];
    robot.deadband_right = (int16_t)values[1];
    robot_unlock();
    return AT_OK;
}

/** Percent of full power; caps the balance output, e.g. for quiet first tests */
static AT_Result_t set_outlimit(const float *values) {
    robot_lock();
    robot.pid.output_limit = values[0] * MOTOR_COMMAND_MAX / 100.0f;
    robot_unlock();
    return AT_OK;
}

static AT_Result_t set_speed(const float *values) {
    robot_lock();
    robot.speed_left = values[0];
    robot.speed_right = values[1];
    /* Percent to signed command; the balance loop overrides it while running */
    motor_set(MOTOR_LEFT, (int16_t)(values[0] * 2.55f));
    motor_set(MOTOR_RIGHT, (int16_t)(values[1] * 2.55f));
    robot_unlock();
    return AT_OK;
}

/* ==========================================================================
 * Execute commands
 * ========================================================================== */

static AT_Result_t exec_enable(void) {
    robot_lock();
    robot_enable();
    robot_unlock();
    return AT_OK;
}

static AT_Result_t exec_disable(void) {
    robot_lock();
    robot_disable();
    robot_unlock();
    return AT_OK;
}

static AT_Result_t exec_stop(void) {
    robot_lock();
    robot_disable();
    robot.target_velocity = 0.0f;
    robot.turn_rate = 0.0f;
    robot.speed_left = 0.0f;
    robot.speed_right = 0.0f;
    robot_unlock();
    return AT_OK;
}

static AT_Result_t exec_default(void) {
    robot_lock();
    robot_restore_defaults();
    robot_unlock();
    return AT_OK;
}

/** AT+SAVE / AT+LOAD: parameter storage in flash is not implemented yet */
static AT_Result_t exec_not_implemented(void) {
    return AT_ERROR;
}

#if AT_CMD_PID_TOGGLE
static AT_Result_t exec_pid_toggle(void) {
    robot_lock();
    robot.pid_enabled = !robot.pid_enabled;
    if (!robot.pid_enabled) {
        pid_reset(&robot.pid);
    }
    robot_unlock();
    return AT_OK;
}

static AT_Result_t exec_pid_on(void) {
    robot_lock();
    robot.pid_enabled = true;
    robot_unlock();
    return AT_OK;
}

static AT_Result_t exec_pid_off(void) {
    robot_lock();
    robot.pid_enabled = false;
    pid_reset(&robot.pid);
    motor_set(MOTOR_LEFT, 0);
    motor_set(MOTOR_RIGHT, 0);
    robot_unlock();
    return AT_OK;
}
#endif // AT_CMD_PID_TOGGLE

/* ==========================================================================
 * Command table
 * ========================================================================== */

static const AT_Command_Def_t robot_commands[] = {
    { .name = "STATUS",   .query = query_status,   .help = AT_HELP("Motors ENABLED/DISABLED, BALANCED/UNBALANCED") },
    { .name = "ACC_X",    .query = query_acc_x,    .help = AT_HELP("X acceleration (g)") },
    { .name = "ACC_Y",    .query = query_acc_y,    .help = AT_HELP("Y acceleration (g)") },
    { .name = "ACC_Z",    .query = query_acc_z,    .help = AT_HELP("Z acceleration (g)") },
    { .name = "GYRO_X",   .query = query_gyro_x,   .help = AT_HELP("X rotation rate (deg/s)") },
    { .name = "GYRO_Y",   .query = query_gyro_y,   .help = AT_HELP("Y rotation rate (deg/s)") },
    { .name = "GYRO_Z",   .query = query_gyro_z,   .help = AT_HELP("Z rotation rate (deg/s)") },
    { .name = "LIVE",     .query = query_live,     .help = AT_HELP("en,bal,tilt,speed,setpoint,out,encL,encR") },
    { .name = "ENC",      .query = query_encoders, .help = AT_HELP("Encoder counts left,right (since boot)") },
    { .name = "ANGLE",    .query = query_angle,    .help = AT_HELP("Filtered tilt (deg, 0 = upright)") },
#if AT_CMD_ALL_QUERY
    { .name = "ALL",      .query = query_all,      .help = AT_HELP("ax,ay,az,gx,gy,gz,angle") },
#endif // AT_CMD_ALL_QUERY
    { .name = "VELOCITY", .query = query_velocity, .set = set_target, .params = 1, .min = -100.0f, .max = 100.0f,
      .help = AT_HELP("Target speed / measured speed, % of full wheel speed") },
    { .name = "TARGET",   .query = query_target,   .set = set_target, .params = 1, .min = -100.0f, .max = 100.0f,
      .help = AT_HELP("Target velocity (stored, not used yet)") },
    { .name = "TURN",     .query = query_turn,     .set = set_turn,   .params = 1, .min = -100.0f, .max = 100.0f,
      .help = AT_HELP("Turn rate, added left / subtracted right") },
    { .name = "SPEED",    .query = query_speed,    .set = set_speed,  .params = 2, .min = -100.0f, .max = 100.0f,
      .help = AT_HELP("Wheel speeds in percent (use with PID off)") },
    { .name = "KP",       .query = query_kp,       .set = set_kp,     .params = 1, .min = 0.0f, .max = FLT_MAX,
      .help = AT_HELP("PID proportional gain") },
    { .name = "KI",       .query = query_ki,       .set = set_ki,     .params = 1, .min = 0.0f, .max = FLT_MAX,
      .help = AT_HELP("PID integral gain") },
    { .name = "KD",       .query = query_kd,       .set = set_kd,     .params = 1, .min = 0.0f, .max = FLT_MAX,
      .help = AT_HELP("PID derivative gain") },
    { .name = "DEADBAND", .query = query_deadband, .set = set_deadband, .params = 2, .integer = true,
      .min = 0.0f, .max = 200.0f, .help = AT_HELP("Motor dead zone left,right (counts of 255)") },
    { .name = "OUTLIMIT", .query = query_outlimit, .set = set_outlimit, .params = 1, .integer = true,
      .min = 20.0f, .max = 100.0f, .help = AT_HELP("Balance output limit, percent of full power") },
#if SPEED_LOOP
    { .name = "VLOOP",    .query = query_vloop,    .set = set_vloop,  .params = 1, .integer = true, .min = 0.0f, .max = 1.0f,
      .help = AT_HELP("Speed loop off/on (keeps the robot in place)") },
    { .name = "VKP",      .query = query_vkp,      .set = set_vkp,    .params = 1, .min = 0.0f, .max = 1.0f,
      .help = AT_HELP("Speed loop gain, deg of lean per % of speed") },
    { .name = "VKI",      .query = query_vki,      .set = set_vki,    .params = 1, .min = 0.0f, .max = 1.0f,
      .help = AT_HELP("Speed loop integral gain") },
#endif // SPEED_LOOP
    { .name = "DGYRO",    .query = query_dgyro,    .set = set_dgyro,  .params = 1, .integer = true, .min = 0.0f, .max = 1.0f,
      .help = AT_HELP("Balance D term from the gyro rate (1) or the angle difference (0)") },
    { .name = "ALPHA",    .query = query_alpha,    .set = set_alpha,  .params = 1, .min = 0.9f, .max = 0.999f,
      .help = AT_HELP("Complementary filter gyro weight (0.9..0.999)") },
    { .name = "SETPOINT", .query = query_setpoint, .set = set_setpoint, .params = 1, .min = -10.0f, .max = 10.0f,
      .help = AT_HELP("Balance target angle (deg): trim, or step tests") },
    { .name = "ENABLE",   .exec = exec_enable,     .help = AT_HELP("Start balancing") },
    { .name = "DISABLE",  .exec = exec_disable,    .help = AT_HELP("Stop balancing, motors in standby") },
    { .name = "STOP",     .exec = exec_stop,       .help = AT_HELP("DISABLE and zero speed, turn, target") },
#if AT_CMD_PID_TOGGLE
    { .name = "PID",      .exec = exec_pid_toggle, .help = AT_HELP("Toggle the balance PID") },
    { .name = "PIDON",    .exec = exec_pid_on,     .help = AT_HELP("Enable the balance PID") },
    { .name = "PIDOFF",   .exec = exec_pid_off,    .help = AT_HELP("Disable the balance PID, zero motors") },
#endif // AT_CMD_PID_TOGGLE
    { .name = "DEFAULT",  .exec = exec_default,    .help = AT_HELP("Default gains, zero turn, target and setpoint") },
    { .name = "SAVE",     .exec = exec_not_implemented, .help = AT_HELP("Not implemented") },
    { .name = "LOAD",     .exec = exec_not_implemented, .help = AT_HELP("Not implemented") },
};

void robot_commands_register(void) {
    at_cmd_register(robot_commands, sizeof(robot_commands) / sizeof(robot_commands[0]));
}
