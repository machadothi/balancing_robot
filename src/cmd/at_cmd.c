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

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/scb.h>

#include "cmd/at_cmd.h"
#include "drivers/uart.h"
#include "config.h"

/* ==========================================================================
 * Float Formatting Helpers (stack-efficient when LOG_ENABLED=0)
 * ========================================================================== */

#if LOG_ENABLED
/* Use standard float formatting (requires more stack) */
#define FORMAT_FLOAT_1(val)     "%.1f", (val)
#define FORMAT_FLOAT_2(val)     "%.2f", (val)
#define FORMAT_FLOAT_3(val)     "%.3f", (val)
#define FORMAT_FLOAT_4(val)     "%.4f", (val)
#else
/* Convert float to integer representation (saves ~400 bytes stack) */
static inline int32_t float_to_int(float val, int scale) {
    return (int32_t)(val * scale + (val >= 0 ? 0.5f : -0.5f));
}

/* Format: integer.fraction (e.g., -1.234 -> "-1.234") */
#define FORMAT_FLOAT_1(val)     "%d.%01d", (int)(val), abs(float_to_int(val, 10) % 10)
#define FORMAT_FLOAT_2(val)     "%d.%02d", (int)(val), abs(float_to_int(val, 100) % 100)
#define FORMAT_FLOAT_3(val)     "%d.%03d", (int)(val), abs(float_to_int(val, 1000) % 1000)
#define FORMAT_FLOAT_4(val)     "%d.%04d", (int)(val), abs(float_to_int(val, 10000) % 10000)
#endif

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

/** Response buffer */
static char response_buffer[AT_RESPONSE_BUFFER_SIZE];

/* ==========================================================================
 * Private Function Prototypes
 * ========================================================================== */

static bool at_parse_command(const char *line, AT_Command_t *cmd);
static void at_handle_test(void);
static void at_handle_query(const AT_Command_t *cmd);
static void at_handle_set(const AT_Command_t *cmd);
static void at_handle_execute(const AT_Command_t *cmd);
static void at_show_help(void);
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

/* ==========================================================================
 * Response Functions
 * ========================================================================== */

void at_cmd_respond_ok(void) {
    uart_println("OK");
    uart_puts(AT_PROMPT);
}

void at_cmd_respond_error(AT_Result_t error) {
    uart_printf("ERROR:%d\r\n", (int)error);
    uart_puts(AT_PROMPT);
}

void at_cmd_respond_data(const char *cmd, const char *fmt, ...) {
    uart_printf("+%s:", cmd);
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(response_buffer, sizeof(response_buffer), fmt, args);
    va_end(args);
    
    uart_println(response_buffer);
    uart_puts(AT_PROMPT);
}

/* ==========================================================================
 * Command Processing
 * ========================================================================== */

void at_cmd_process(const char *line, uint16_t length) {
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
        uart_println("ERROR:Invalid command (must start with AT)");
        uart_puts(AT_PROMPT);
        return;
    }
    
    /* Parse and process */
    AT_Command_t cmd;
    if (!at_parse_command(cmd_start, &cmd)) {
        /* Extended commands must use AT+CMD format */
        uart_println("ERROR:Invalid syntax (use AT+CMD?, AT+CMD=val, or AT+CMD)");
        uart_puts(AT_PROMPT);
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
        
        /* Try to parse as number */
        char *endptr;
        cmd->param_float = strtof(cmd->param, &endptr);
        cmd->param_int = (int32_t)strtol(cmd->param, &endptr, 10);
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
    char *comma = strchr(param, ',');
    if (comma == NULL) {
        return false;
    }
    
    *val1 = strtof(param, NULL);
    *val2 = strtof(comma + 1, NULL);
    return true;
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
    
    /* Match command */
    if (strcmp(cmd->cmd, "VERSION") == 0) {
        at_cmd_respond_data("VERSION", "%s", FIRMWARE_VERSION);
    }
    else if (strcmp(cmd->cmd, "STATUS") == 0) {
        at_cmd_respond_data("STATUS", "%s,%s",
            robot_state->motors_enabled ? "ENABLED" : "DISABLED",
            robot_state->is_balanced ? "BALANCED" : "UNBALANCED");
    }
    else if (strcmp(cmd->cmd, "ACC_X") == 0) {
        at_cmd_respond_data("ACC_X", FORMAT_FLOAT_3(robot_state->acc_x));
    }
    else if (strcmp(cmd->cmd, "ACC_Y") == 0) {
        at_cmd_respond_data("ACC_Y", FORMAT_FLOAT_3(robot_state->acc_y));
    }
    else if (strcmp(cmd->cmd, "ACC_Z") == 0) {
        at_cmd_respond_data("ACC_Z", FORMAT_FLOAT_3(robot_state->acc_z));
    }
    else if (strcmp(cmd->cmd, "GYRO_X") == 0) {
        at_cmd_respond_data("GYRO_X", FORMAT_FLOAT_3(robot_state->gyro_x));
    }
    else if (strcmp(cmd->cmd, "GYRO_Y") == 0) {
        at_cmd_respond_data("GYRO_Y", FORMAT_FLOAT_3(robot_state->gyro_y));
    }
    else if (strcmp(cmd->cmd, "GYRO_Z") == 0) {
        at_cmd_respond_data("GYRO_Z", FORMAT_FLOAT_3(robot_state->gyro_z));
    }
    else if (strcmp(cmd->cmd, "ANGLE") == 0) {
        at_cmd_respond_data("ANGLE", FORMAT_FLOAT_2(robot_state->angle));
    }
    else if (strcmp(cmd->cmd, "VELOCITY") == 0) {
        at_cmd_respond_data("VELOCITY", FORMAT_FLOAT_2(robot_state->velocity));
    }
    else if (strcmp(cmd->cmd, "TARGET") == 0) {
        at_cmd_respond_data("TARGET", FORMAT_FLOAT_2(robot_state->target_velocity));
    }
    else if (strcmp(cmd->cmd, "TURN") == 0) {
        at_cmd_respond_data("TURN", FORMAT_FLOAT_2(robot_state->turn_rate));
    }
    else if (strcmp(cmd->cmd, "KP") == 0) {
        at_cmd_respond_data("KP", FORMAT_FLOAT_4(robot_state->kp));
    }
    else if (strcmp(cmd->cmd, "KI") == 0) {
        at_cmd_respond_data("KI", FORMAT_FLOAT_4(robot_state->ki));
    }
    else if (strcmp(cmd->cmd, "KD") == 0) {
        at_cmd_respond_data("KD", FORMAT_FLOAT_4(robot_state->kd));
    }
    else if (strcmp(cmd->cmd, "SPEED") == 0) {
#if LOG_ENABLED
        at_cmd_respond_data("SPEED", "%.1f,%.1f", 
            robot_state->speed_left, robot_state->speed_right);
#else
        at_cmd_respond_data("SPEED", "%d.%01d,%d.%01d", 
            (int)robot_state->speed_left, abs(float_to_int(robot_state->speed_left, 10) % 10),
            (int)robot_state->speed_right, abs(float_to_int(robot_state->speed_right, 10) % 10));
#endif
    }
    else if (strcmp(cmd->cmd, "ALL") == 0) {
        /* Return all sensor data in CSV format */
#if LOG_ENABLED
        at_cmd_respond_data("ALL", "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f",
            robot_state->acc_x, robot_state->acc_y, robot_state->acc_z,
            robot_state->gyro_x, robot_state->gyro_y, robot_state->gyro_z,
            robot_state->angle);
#else
        at_cmd_respond_data("ALL", "%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%03d,%d.%02d",
            (int)robot_state->acc_x, abs(float_to_int(robot_state->acc_x, 1000) % 1000),
            (int)robot_state->acc_y, abs(float_to_int(robot_state->acc_y, 1000) % 1000),
            (int)robot_state->acc_z, abs(float_to_int(robot_state->acc_z, 1000) % 1000),
            (int)robot_state->gyro_x, abs(float_to_int(robot_state->gyro_x, 1000) % 1000),
            (int)robot_state->gyro_y, abs(float_to_int(robot_state->gyro_y, 1000) % 1000),
            (int)robot_state->gyro_z, abs(float_to_int(robot_state->gyro_z, 1000) % 1000),
            (int)robot_state->angle, abs(float_to_int(robot_state->angle, 100) % 100));
#endif
    }
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
        if (set_callback("SPEED", left, right)) {
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
        if (set_callback("VELOCITY", cmd->param_float, 0.0f)) {
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
        if (set_callback("TURN", cmd->param_float, 0.0f)) {
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
        if (set_callback("KP", cmd->param_float, 0.0f)) {
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
        if (set_callback("KI", cmd->param_float, 0.0f)) {
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
        if (set_callback("KD", cmd->param_float, 0.0f)) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else {
        at_cmd_respond_error(AT_ERROR_UNKNOWN_CMD);
    }
}

/**
 * @brief Handle execute commands (AT+CMD)
 */
static void at_handle_execute(const AT_Command_t *cmd) {
    /* Check for callback */
    if (exec_callback == NULL && strcmp(cmd->cmd, "HELP") != 0) {
        at_cmd_respond_error(AT_ERROR_NOT_READY);
        return;
    }
    
    /* Handle known execute commands */
    if (strcmp(cmd->cmd, "ENABLE") == 0) {
        if (exec_callback("ENABLE")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "DISABLE") == 0) {
        if (exec_callback("DISABLE")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "STOP") == 0) {
        if (exec_callback("STOP")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "RESET") == 0) {
        at_cmd_respond_ok();
        /* Small delay to allow response to be sent */
        vTaskDelay(pdMS_TO_TICKS(100));
        /* Trigger system reset */
        scb_reset_system();
    }
    else if (strcmp(cmd->cmd, "SAVE") == 0) {
        if (exec_callback("SAVE")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "LOAD") == 0) {
        if (exec_callback("LOAD")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "DEFAULT") == 0) {
        if (exec_callback("DEFAULT")) {
            at_cmd_respond_ok();
        } else {
            at_cmd_respond_error(AT_ERROR);
        }
    }
    else if (strcmp(cmd->cmd, "HELP") == 0) {
        at_show_help();
    }
    else {
        at_cmd_respond_error(AT_ERROR_UNKNOWN_CMD);
    }
}

/**
 * @brief Show help message
 */
static void at_show_help(void) {
    uart_println("");
    uart_println("+HELP:AT Command Reference");
    uart_println("  AT              Test connection");
    uart_println("  AT+VERSION?     Firmware version");
    uart_println("  AT+STATUS?      Robot status");
    uart_println("  AT+ALL?         All sensor data");
    uart_println("  AT+ACC_X?       X acceleration");
    uart_println("  AT+ACC_Y?       Y acceleration");
    uart_println("  AT+ACC_Z?       Z acceleration");
    uart_println("  AT+GYRO_X?      X rotation rate");
    uart_println("  AT+GYRO_Y?      Y rotation rate");
    uart_println("  AT+GYRO_Z?      Z rotation rate");
    uart_println("  AT+ANGLE?       Tilt angle");
    uart_println("  AT+VELOCITY?    Current velocity");
    uart_println("  AT+VELOCITY=n   Set target (-100..100)");
    uart_println("  AT+SPEED=l,r    Set wheel speeds (-100..100)");
    uart_println("  AT+SPEED?       Get wheel speeds");
    uart_println("  AT+TURN=n       Set turn rate (-100..100)");
    uart_println("  AT+KP?/=n       PID proportional");
    uart_println("  AT+KI?/=n       PID integral");
    uart_println("  AT+KD?/=n       PID derivative");
    uart_println("  AT+ENABLE       Enable motors");
    uart_println("  AT+DISABLE      Disable motors");
    uart_println("  AT+STOP         Emergency stop");
    uart_println("  AT+RESET        System reset");
    uart_println("  AT+HELP         This help");
    at_cmd_respond_ok();
}

/* ==========================================================================
 * Task
 * ========================================================================== */

void at_cmd_task(void *args) {
    (void)args;
    
    /* Show initial prompt (at_cmd_init already called from main) */
    uart_println("");
    uart_println("AT Command Interface Ready");
    uart_println("Type AT+HELP for commands");
    uart_puts(AT_PROMPT);
    
    /* This task can be deleted - callback handles everything */
    vTaskDelete(NULL);
}
