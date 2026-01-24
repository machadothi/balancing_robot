/**
 * @file at_cmd.h
 * @brief AT Command Parser Interface
 * 
 * Implements a standard AT command interface for robot control via UART.
 * Compatible with serial terminals and Raspberry Pi communication.
 * 
 * Command Format:
 *   AT                     - Test command, returns "OK"
 *   AT+<CMD>?              - Query command (read value)
 *   AT+<CMD>=<value>       - Set command (write value)
 *   AT+<CMD>               - Execute command (action)
 * 
 * Supported Commands:
 *   AT+VERSION?            - Get firmware version
 *   AT+STATUS?             - Get robot status
 *   AT+ACC_X?              - Get X acceleration (g)
 *   AT+ACC_Y?              - Get Y acceleration (g)
 *   AT+ACC_Z?              - Get Z acceleration (g)
 *   AT+GYRO_X?             - Get X rotation rate (deg/s)
 *   AT+GYRO_Y?             - Get Y rotation rate (deg/s)
 *   AT+GYRO_Z?             - Get Z rotation rate (deg/s)
 *   AT+ANGLE?              - Get current tilt angle (degrees)
 *   AT+VELOCITY?           - Get current velocity
 *   AT+VELOCITY=<val>      - Set target velocity (-100 to 100)
 *   AT+TURN=<val>          - Set turn rate (-100 to 100)
 *   AT+KP?                 - Get PID proportional gain
 *   AT+KP=<val>            - Set PID proportional gain
 *   AT+KI?                 - Get PID integral gain
 *   AT+KI=<val>            - Set PID integral gain
 *   AT+KD?                 - Get PID derivative gain
 *   AT+KD=<val>            - Set PID derivative gain
 *   AT+SPEED=<l>,<r>       - Set left/right wheel speed (-100 to 100 each)
 *   AT+SPEED?              - Get current wheel speeds
 *   AT+ENABLE              - Enable motors
 *   AT+DISABLE             - Disable motors
 *   AT+STOP                - Emergency stop
 *   AT+RESET               - Software reset
 *   AT+SAVE                - Save settings to flash
 *   AT+LOAD                - Load settings from flash
 *   AT+DEFAULT             - Restore default settings
 *   AT+HELP                - List available commands
 * 
 * Response Format:
 *   OK                     - Command successful (no data)
 *   +<CMD>:<value>         - Query response with data
 *   ERROR:<code>           - Error with numeric code
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef AT_CMD_H
#define AT_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief AT command result codes
 */
typedef enum {
    AT_OK = 0,              /**< Command successful */
    AT_ERROR,               /**< General error */
    AT_ERROR_UNKNOWN_CMD,   /**< Unknown command */
    AT_ERROR_INVALID_PARAM, /**< Invalid parameter */
    AT_ERROR_RANGE,         /**< Parameter out of range */
    AT_ERROR_NOT_READY,     /**< System not ready */
    AT_ERROR_BUSY,          /**< System busy */
} AT_Result_t;

/**
 * @brief AT command type
 */
typedef enum {
    AT_TYPE_TEST,           /**< AT (basic test) */
    AT_TYPE_QUERY,          /**< AT+CMD? (read) */
    AT_TYPE_SET,            /**< AT+CMD=value (write) */
    AT_TYPE_EXECUTE,        /**< AT+CMD (action) */
} AT_CmdType_t;

/**
 * @brief Parsed AT command structure
 */
typedef struct {
    AT_CmdType_t type;          /**< Command type */
    char         cmd[24];       /**< Command name (e.g., "VELOCITY") */
    char         param[64];     /**< Parameter string (for SET commands) */
    float        param_float;   /**< Parsed float parameter */
    int32_t      param_int;     /**< Parsed integer parameter */
} AT_Command_t;

/**
 * @brief Robot control values accessible via AT commands
 */
typedef struct {
    /* IMU readings */
    float acc_x;            /**< X acceleration (g) */
    float acc_y;            /**< Y acceleration (g) */
    float acc_z;            /**< Z acceleration (g) */
    float gyro_x;           /**< X rotation rate (deg/s) */
    float gyro_y;           /**< Y rotation rate (deg/s) */
    float gyro_z;           /**< Z rotation rate (deg/s) */
    float angle;            /**< Tilt angle (degrees) */
    
    /* Control parameters */
    float velocity;         /**< Current velocity */
    float target_velocity;  /**< Target velocity (-100 to 100) */
    float turn_rate;        /**< Turn rate (-100 to 100) */
    
    /* PID gains */
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    
    /* Wheel speeds */
    float speed_left;       /**< Left wheel speed (-100 to 100) */
    float speed_right;      /**< Right wheel speed (-100 to 100) */
    
    /* Status */
    bool  motors_enabled;   /**< Motors enabled flag */
    bool  pid_enabled;      /**< PID controller enabled flag */
    bool  is_balanced;      /**< Robot is balanced */
} AT_RobotState_t;

/**
 * @brief Callback for setting values
 * 
 * @param param     Parameter name (e.g., "VELOCITY")
 * @param value     New value
 * @param value2    Second value (for dual-parameter commands like SPEED)
 * @return true if successful
 */
typedef bool (*AT_SetCallback_t)(const char *param, float value, float value2);

/**
 * @brief Callback for execute commands
 * 
 * @param cmd   Command name (e.g., "ENABLE", "STOP")
 * @return true if successful
 */
typedef bool (*AT_ExecCallback_t)(const char *cmd);

/* ==========================================================================
 * Initialization
 * ========================================================================== */

/**
 * @brief Initialize AT command parser
 * 
 * Sets up the command parser and registers it as UART RX callback.
 */
void at_cmd_init(void);

/**
 * @brief Register robot state pointer
 * 
 * The AT parser will read values from this structure for query commands.
 * 
 * @param state     Pointer to robot state (must remain valid)
 */
void at_cmd_set_state(AT_RobotState_t *state);

/**
 * @brief Register set callback
 * 
 * Called when a SET command is received (AT+CMD=value).
 * 
 * @param callback  Function to handle set operations
 */
void at_cmd_set_callback(AT_SetCallback_t callback);

/**
 * @brief Register execute callback
 * 
 * Called when an execute command is received (AT+CMD).
 * 
 * @param callback  Function to handle execute operations
 */
void at_cmd_exec_callback(AT_ExecCallback_t callback);

/* ==========================================================================
 * Command Processing
 * ========================================================================== */

/**
 * @brief Process a received AT command line
 * 
 * Parses the command and generates appropriate response.
 * This is called automatically when registered as UART callback.
 * 
 * @param line      Command line (without line terminator)
 * @param length    Line length
 */
void at_cmd_process(const char *line, uint16_t length);

/**
 * @brief Send an OK response
 */
void at_cmd_respond_ok(void);

/**
 * @brief Send an error response
 * 
 * @param error     Error code
 */
void at_cmd_respond_error(AT_Result_t error);

/**
 * @brief Send a data response
 * 
 * Format: +CMD:value
 * 
 * @param cmd       Command name
 * @param fmt       Printf format for value
 * @param ...       Value arguments
 */
void at_cmd_respond_data(const char *cmd, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* ==========================================================================
 * AT Command Task
 * ========================================================================== */

/**
 * @brief AT command processing task
 * 
 * Handles command processing in task context.
 * 
 * @param args  Task arguments (unused)
 */
void at_cmd_task(void *args);

#ifdef __cplusplus
}
#endif

#endif /* AT_CMD_H */
