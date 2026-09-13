/**
 * @file at_cmd.c
 * @brief AT Command Parser Implementation
 * 
 * Standard AT command interface for robot control.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/scb.h>

#include "cmd/at_cmd.h"
#include "drivers/uart.h"
#include "config.h"
#include "util/fmt.h"

/* ==========================================================================
 * Float Formatting Helpers (stack-efficient when LOGGING=0)
 * ========================================================================== */

/* Integer-printf formatting via fmt_fixed(): float printf needs ~400 bytes
 * more stack. Each compound literal is its own buffer, alive for the whole call. */
#define FORMAT_FIXED(val, decimals)  fmt_fixed((char[16]){0}, 16, (val), (decimals))
#define FORMAT_FLOAT_1(val)     "%s", FORMAT_FIXED(val, 1)
#define FORMAT_FLOAT_2(val)     "%s", FORMAT_FIXED(val, 2)
#define FORMAT_FLOAT_3(val)     "%s", FORMAT_FIXED(val, 3)
#define FORMAT_FLOAT_4(val)     "%s", FORMAT_FIXED(val, 4)

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Firmware version string */
#define FIRMWARE_VERSION    "1.0.0"

/** Response buffer size */
#define AT_RESPONSE_BUFFER_SIZE     128

/** Prompt string */
#define AT_PROMPT           "> "

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** Pointer to robot state (provided by application) */
static AT_RobotState_t *robot_state = NULL;

/** Set callback */
static AT_SetCallback_t set_callback = NULL;

/** Execute callback */
static AT_ExecCallback_t exec_callback = NULL;

/** Robot state lock hooks (optional) */
static AT_LockCallback_t lock_callback = NULL;
static AT_LockCallback_t unlock_callback = NULL;

/** Port the command being processed came from; uart_rx_task runs commands one at a time */
static UART_Port_t reply_port = (UART_Port_t)0;

/* ==========================================================================
 * Private Function Prototypes
 * ========================================================================== */

static bool at_parse_command(const char *line, AT_Command_t *cmd);
static void at_handle_test(void);
static void at_handle_query(const AT_Command_t *cmd);
static void at_handle_set(const AT_Command_t *cmd);
static void at_handle_execute(const AT_Command_t *cmd);
#if AT_CMD_HELP
static void at_show_help(void);
#endif // AT_CMD_HELP
static void str_to_upper(char *s);

/* ==========================================================================
 * Initialization
 * ========================================================================== */

void at_cmd_init(void) {
    /* Register ourselves as the UART RX callback */
    uart_set_rx_callback(at_cmd_process);
}

void at_cmd_set_state(AT_RobotState_t *state) {
    robot_state = state;
}

void at_cmd_set_callback(AT_SetCallback_t callback) {
    set_callback = callback;
}

void at_cmd_exec_callback(AT_ExecCallback_t callback) {
    exec_callback = callback;
}

void at_cmd_set_lock(AT_LockCallback_t lock, AT_LockCallback_t unlock) {
    lock_callback = lock;
    unlock_callback = unlock;
}

static void at_lock(void) {
    if (lock_callback) {
        lock_callback();
    }
}

static void at_unlock(void) {
    if (unlock_callback) {
        unlock_callback();
    }
}

/* The lock covers only the callback: responses are printed afterwards, so a
 * full UART queue can never stall the control loop waiting on the lock */
static bool at_call_set(const char *param, float value, float value2) {
    at_lock();
    bool ok = set_callback(param, value, value2);
    at_unlock();
    return ok;
}

static AT_Result_t at_call_exec(const char *cmd) {
    at_lock();
    AT_Result_t result = exec_callback(cmd);
    at_unlock();
    return result;
}

/* ==========================================================================
 * Response Functions
 * ========================================================================== */

/**
 * @brief Send the last line of a reply plus the prompt as one atomic write
 *
 * Separate writes would let a telemetry line land in the middle of a reply.
 */
static void at_reply(const char *text) {
    char line[AT_RESPONSE_BUFFER_SIZE];
    snprintf(line, sizeof(line), "%s\r\n" AT_PROMPT, text);
    (void)uart_puts(reply_port, line);
}

void at_cmd_respond_ok(void) {
    at_reply("OK");
}

void at_cmd_respond_error(AT_Result_t error) {
    char text[16];
    snprintf(text, sizeof(text), "ERROR:%d", (int)error);
    at_reply(text);
}

void at_cmd_respond_data(const char *cmd, const char *fmt, ...) {
    char text[AT_RESPONSE_BUFFER_SIZE];
    int prefix = snprintf(text, sizeof(text), "+%s:", cmd);

    va_list args;
    va_start(args, fmt);
    vsnprintf(text + prefix, sizeof(text) - (size_t)prefix, fmt, args);
    va_end(args);

    at_reply(text);
}

/* ==========================================================================
 * Command Processing
 * ========================================================================== */

void at_cmd_process(UART_Port_t port, const char *line, uint16_t length) {
    reply_port = port;

    if (line == NULL || length == 0) {
        return;
    }
    
    /* Copy to modifiable buffer */
    char buffer[UART_RX_LINE_SIZE];
    if (length >= sizeof(buffer)) {
        length = sizeof(buffer) - 1;
    }
    memcpy(buffer, line, length);
    buffer[length] = '\0';
    
    /* Convert to uppercase for case-insensitive parsing */
    str_to_upper(buffer);
    
    /* Trim leading whitespace */
    char *cmd_start = buffer;
    while (*cmd_start == ' ' || *cmd_start == '\t') {
        cmd_start++;
    }
    
    /* Check for AT prefix */
    if (strncmp(cmd_start, "AT", 2) != 0) {
        /* Not an AT command - show error */
        at_reply("ERROR:Invalid command (must start with AT)");
        return;
    }
    
    /* Parse and process */
    AT_Command_t cmd;
    if (!at_parse_command(cmd_start, &cmd)) {
        /* Extended commands must use AT+CMD format */
        at_reply("ERROR:Invalid syntax (use AT+CMD?, AT+CMD=val, or AT+CMD)");
        return;
    }
    
    /* Dispatch based on type */
    switch (cmd.type) {
        case AT_TYPE_TEST:
            at_handle_test();
            break;
        case AT_TYPE_QUERY:
            at_handle_query(&cmd);
            break;
        case AT_TYPE_SET:
            at_handle_set(&cmd);
            break;
        case AT_TYPE_EXECUTE:
            at_handle_execute(&cmd);
            break;
    }
}

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Convert string to uppercase
 */
static void str_to_upper(char *s) {
    while (*s) {
        *s = toupper((unsigned char)*s);
        s++;
    }
}

/**
 * @brief Parse AT command line
 * 
 * @param line  Command line starting with "AT"
 * @param cmd   Output parsed command
 * @return true if parsed successfully
 */
static bool at_parse_command(const char *line, AT_Command_t *cmd) {
    memset(cmd, 0, sizeof(*cmd));
    
    /* Skip "AT" */
    line += 2;
    
    /* Check for basic "AT" test command */
    if (*line == '\0') {
        cmd->type = AT_TYPE_TEST;
        return true;
    }
    
    /* Must have "+" for extended commands */
    if (*line != '+') {
        return false;
    }
    line++;
    
    /* Extract command name (until '?', '=', or end) */
    const char *cmd_end = line;
    while (*cmd_end && *cmd_end != '?' && *cmd_end != '=') {
        cmd_end++;
    }
    
    size_t cmd_len = cmd_end - line;
    if (cmd_len == 0 || cmd_len >= sizeof(cmd->cmd)) {
        return false;
    }
    
    memcpy(cmd->cmd, line, cmd_len);
    cmd->cmd[cmd_len] = '\0';
    
    /* Determine type */
    if (*cmd_end == '?') {
        cmd->type = AT_TYPE_QUERY;
    } else if (*cmd_end == '=') {
        cmd->type = AT_TYPE_SET;
        
        /* Extract parameter */
        const char *param_start = cmd_end + 1;
        size_t param_len = strlen(param_start);
        if (param_len >= sizeof(cmd->param)) {
            param_len = sizeof(cmd->param) - 1;
        }
        memcpy(cmd->param, param_start, param_len);
        cmd->param[param_len] = '\0';
        
        /* Numeric only if the whole parameter parsed and is finite: otherwise
         * "abc" would become 0 and "nan" would slip past range checks */
        char *endptr;
        cmd->param_float = strtof(cmd->param, &endptr);
        cmd->param_is_number = (endptr != cmd->param) && (*endptr == '\0') &&
                               isfinite(cmd->param_float);
        cmd->param_int = (int32_t)strtol(cmd->param, NULL, 10);
    } else {
        cmd->type = AT_TYPE_EXECUTE;
    }
    
    return true;
}

/**
 * @brief Parse comma-separated parameters
 * 
 * @param param  Input parameter string (e.g., "50,-30")
 * @param val1   Output first value
 * @param val2   Output second value
 * @return true if two values parsed successfully
 */
static bool parse_dual_param(const char *param, float *val1, float *val2) {
    char *end1;
    char *end2;

    *val1 = strtof(param, &end1);
    if (end1 == param || *end1 != ',') {
        return false;
    }

    *val2 = strtof(end1 + 1, &end2);
    return (end2 != end1 + 1) && (*end2 == '\0') && isfinite(*val1) && isfinite(*val2);
}

/**
 * @brief Handle basic AT test command
 */
static void at_handle_test(void) {
    at_cmd_respond_ok();
}

/**
 * @brief Handle query commands (AT+CMD?)
 */
static void at_handle_query(const AT_Command_t *cmd) {
    /* Check if state is available */
    if (robot_state == NULL) {
        at_cmd_respond_error(AT_ERROR_NOT_READY);
        return;
    }

    /* Consistent snapshot, so formatting never runs with the lock held */
    at_lock();
    const AT_RobotState_t state_copy = *robot_state;
    at_unlock();
    const AT_RobotState_t *state = &state_copy;

    /* Match command */
    if (strcmp(cmd->cmd, "VERSION") == 0) {
        at_cmd_respond_data("VERSION", "%s", FIRMWARE_VERSION);
    }
    else if (strcmp(cmd->cmd, "STATUS") == 0) {
        at_cmd_respond_data("STATUS", "%s,%s",
            state->motors_enabled ? "ENABLED" : "DISABLED",
            state->is_balanced ? "BALANCED" : "UNBALANCED");
    }
    else if (strcmp(cmd->cmd, "ACC_X") == 0) {
        at_cmd_respond_data("ACC_X", FORMAT_FLOAT_3(state->acc_x));
    }
    else if (strcmp(cmd->cmd, "ACC_Y") == 0) {
        at_cmd_respond_data("ACC_Y", FORMAT_FLOAT_3(state->acc_y));
    }
    else if (strcmp(cmd->cmd, "ACC_Z") == 0) {
        at_cmd_respond_data("ACC_Z", FORMAT_FLOAT_3(state->acc_z));
    }
    else if (strcmp(cmd->cmd, "GYRO_X") == 0) {
        at_cmd_respond_data("GYRO_X", FORMAT_FLOAT_3(state->gyro_x));
    }
    else if (strcmp(cmd->cmd, "GYRO_Y") == 0) {
        at_cmd_respond_data("GYRO_Y", FORMAT_FLOAT_3(state->gyro_y));
    }
    else if (strcmp(cmd->cmd, "GYRO_Z") == 0) {
        at_cmd_respond_data("GYRO_Z", FORMAT_FLOAT_3(state->gyro_z));
    }
    else if (strcmp(cmd->cmd, "ANGLE") == 0) {
        at_cmd_respond_data("ANGLE", FORMAT_FLOAT_2(state->angle));
    }
    else if (strcmp(cmd->cmd, "VELOCITY") == 0) {
        at_cmd_respond_data("VELOCITY", FORMAT_FLOAT_2(state->velocity));
    }
    else if (strcmp(cmd->cmd, "TARGET") == 0) {
        at_cmd_respond_data("TARGET", FORMAT_FLOAT_2(state->target_velocity));
    }
    else if (strcmp(cmd->cmd, "TURN") == 0) {
        at_cmd_respond_data("TURN", FORMAT_FLOAT_2(state->turn_rate));
    }
    else if (strcmp(cmd->cmd, "KP") == 0) {
        at_cmd_respond_data("KP", FORMAT_FLOAT_4(state->kp));
    }
    else if (strcmp(cmd->cmd, "KI") == 0) {
        at_cmd_respond_data("KI", FORMAT_FLOAT_4(state->ki));
    }
    else if (strcmp(cmd->cmd, "KD") == 0) {
        at_cmd_respond_data("KD", FORMAT_FLOAT_4(state->kd));
    }
    else if (strcmp(cmd->cmd, "SPEED") == 0) {
        at_cmd_respond_data("SPEED", "%s,%s",
            FORMAT_FIXED(state->speed_left, 1), FORMAT_FIXED(state->speed_right, 1));
    }
#if AT_CMD_ALL_QUERY
    else if (strcmp(cmd->cmd, "ALL") == 0) {
        /* Return all sensor data in CSV format */
        at_cmd_respond_data("ALL", "%s,%s,%s,%s,%s,%s,%s",
            FORMAT_FIXED(state->acc_x, 3), FORMAT_FIXED(state->acc_y, 3),
            FORMAT_FIXED(state->acc_z, 3), FORMAT_FIXED(state->gyro_x, 3),
            FORMAT_FIXED(state->gyro_y, 3), FORMAT_FIXED(state->gyro_z, 3),
            FORMAT_FIXED(state->angle, 2));
    }
#endif // AT_CMD_ALL_QUERY
    else {
        at_cmd_respond_error(AT_ERROR_UNKNOWN_CMD);
    }
}

/**
 * @brief Handle set commands (AT+CMD=value)
 */
static void at_handle_set(const AT_Command_t *cmd) {
    /* Check for callback */
    if (set_callback == NULL) {
        at_cmd_respond_error(AT_ERROR_NOT_READY);
        return;
    }
    
    /* Validate parameter */
    if (strlen(cmd->param) == 0) {
        at_cmd_respond_error(AT_ERROR_INVALID_PARAM);
        return;
    }

    /* Every set command except SPEED takes a single finite number */
    if (strcmp(cmd->cmd, "SPEED") != 0 && !cmd->param_is_number) {
        at_cmd_respond_error(AT_ERROR_INVALID_PARAM);
        return;
    }
    
    /* Handle known set commands */
    if (strcmp(cmd->cmd, "SPEED") == 0) {
        /* Speed: left,right (-100 to 100 each) */
        float left, right;
        if (!parse_dual_param(cmd->param, &left, &right)) {
            at_cmd_respond_error(AT_ERROR_INVALID_PARAM);
            return;
        }
        if (left < -100.0f || left > 100.0f || 
            right < -100.0f || right > 100.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("SPEED", left, right)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "VELOCITY") == 0 ||
        strcmp(cmd->cmd, "TARGET") == 0) {
        /* Velocity: -100 to 100 */
        if (cmd->param_float < -100.0f || cmd->param_float > 100.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("VELOCITY", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "TURN") == 0) {
        /* Turn rate: -100 to 100 */
        if (cmd->param_float < -100.0f || cmd->param_float > 100.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("TURN", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "KP") == 0) {
        if (cmd->param_float < 0.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("KP", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "KI") == 0) {
        if (cmd->param_float < 0.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("KI", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "KD") == 0) {
        if (cmd->param_float < 0.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("KD", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
#if TELEMETRY
    else if (strcmp(cmd->cmd, "STREAM") == 0) {
        if (cmd->param_float != 0.0f && cmd->param_float != 1.0f) {
            at_cmd_respond_error(AT_ERROR_RANGE);
            return;
        }
        if (at_call_set("STREAM", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
#endif // TELEMETRY
    else {
        at_cmd_respond_error(AT_ERROR_UNKNOWN_CMD);
    }
}

/**
 * @brief Handle execute commands (AT+CMD)
 */
static void at_handle_execute(const AT_Command_t *cmd) {
    /* RESET and HELP are the parser's own; every other command belongs to the application */
    if (strcmp(cmd->cmd, "RESET") == 0) {
        at_cmd_respond_ok();
        /* Small delay to allow response to be sent */
        vTaskDelay(pdMS_TO_TICKS(100));
        /* Trigger system reset */
        scb_reset_system();
    }
#if AT_CMD_HELP
    else if (strcmp(cmd->cmd, "HELP") == 0) {
        at_show_help();
    }
#endif // AT_CMD_HELP
    else if (exec_callback == NULL) {
        at_cmd_respond_error(AT_ERROR_NOT_READY);
    }
    else {
        AT_Result_t result = at_call_exec(cmd->cmd);
        if (result == AT_OK) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(result);
        }
    }
}

/**
 * @brief Show help message
 */
#if AT_CMD_HELP
static void at_println(const char *text) {
    char line[AT_RESPONSE_BUFFER_SIZE];
    snprintf(line, sizeof(line), "%s\r\n", text);
    (void)uart_puts(reply_port, line);
}

static void at_show_help(void) {
    at_println("");
    at_println("+HELP:AT Command Reference");
    at_println("  AT              Test connection");
    at_println("  AT+VERSION?     Firmware version");
    at_println("  AT+STATUS?      Robot status");
#if AT_CMD_ALL_QUERY
    at_println("  AT+ALL?         All sensor data");
#endif // AT_CMD_ALL_QUERY
    at_println("  AT+ACC_X?       X acceleration");
    at_println("  AT+ACC_Y?       Y acceleration");
    at_println("  AT+ACC_Z?       Z acceleration");
    at_println("  AT+GYRO_X?      X rotation rate");
    at_println("  AT+GYRO_Y?      Y rotation rate");
    at_println("  AT+GYRO_Z?      Z rotation rate");
    at_println("  AT+ANGLE?       Tilt angle");
    at_println("  AT+VELOCITY?    Current velocity");
    at_println("  AT+VELOCITY=n   Set target (-100..100)");
    at_println("  AT+SPEED=l,r    Set wheel speeds (-100..100)");
    at_println("  AT+SPEED?       Get wheel speeds");
    at_println("  AT+TURN=n       Set turn rate (-100..100)");
    at_println("  AT+KP?/=n       PID proportional");
    at_println("  AT+KI?/=n       PID integral");
    at_println("  AT+KD?/=n       PID derivative");
#if TELEMETRY
    at_println("  AT+STREAM=0|1   Telemetry on the USB console");
#endif // TELEMETRY
    at_println("  AT+ENABLE       Enable motors");
    at_println("  AT+DISABLE      Disable motors");
#if AT_CMD_PID_TOGGLE
    at_println("  AT+PID/PIDON/PIDOFF  Toggle/enable/disable PID");
#endif // AT_CMD_PID_TOGGLE
    at_println("  AT+STOP         Emergency stop");
    at_println("  AT+RESET        System reset");
    at_println("  AT+HELP         This help");
    at_cmd_respond_ok();
}
#endif // AT_CMD_HELP
