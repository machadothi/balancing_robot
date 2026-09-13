/**
 * @file at_cmd.c
 * @brief Generic, table-driven AT command parser
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/cm3/scb.h>

#include "config.h"
#include "cmd/at_cmd.h"
#include "drivers/uart.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

#define FIRMWARE_VERSION            "1.0.0"
#define AT_RESPONSE_BUFFER_SIZE     128
#define AT_PROMPT                   "> "
#define AT_NAME_SIZE                24
#define AT_PARAM_SIZE               64

/** Parser, robot and telemetry tables, plus room for one more module */
#define AT_MAX_TABLES               4

typedef enum {
    AT_TYPE_TEST,
    AT_TYPE_QUERY,
    AT_TYPE_SET,
    AT_TYPE_EXECUTE,
} AT_CmdType_t;

typedef struct {
    AT_CmdType_t type;
    char name[AT_NAME_SIZE];
    char param[AT_PARAM_SIZE];
} AT_Command_t;

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

static const AT_Command_Def_t *tables[AT_MAX_TABLES];
static size_t table_sizes[AT_MAX_TABLES];
static size_t table_count = 0;

/** Port of the command being processed; uart_rx_task runs commands one at a time */
static UART_Port_t reply_port = (UART_Port_t)0;

/* ==========================================================================
 * Replies
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

static void at_reply_result(AT_Result_t result) {
    if (result == AT_OK) {
        at_reply("OK");
    } else {
        char text[16];
        snprintf(text, sizeof(text), "ERROR:%d", (int)result);
        at_reply(text);
    }
}

/* ==========================================================================
 * Parsing
 * ========================================================================== */

static void str_to_upper(char *s) {
    for (; *s; s++) {
        *s = (char)toupper((unsigned char)*s);
    }
}

/**
 * @brief Split "AT", "AT+NAME?", "AT+NAME=param" or "AT+NAME"
 * @return false on malformed syntax
 */
static bool at_parse_command(const char *line, AT_Command_t *cmd) {
    memset(cmd, 0, sizeof(*cmd));
    line += 2;  /* "AT" */

    if (*line == '\0') {
        cmd->type = AT_TYPE_TEST;
        return true;
    }
    if (*line++ != '+') {
        return false;
    }

    const char *end = line;
    while (*end && *end != '?' && *end != '=') {
        end++;
    }
    size_t len = (size_t)(end - line);
    if (len == 0 || len >= sizeof(cmd->name)) {
        return false;
    }
    memcpy(cmd->name, line, len);

    if (*end == '?') {
        cmd->type = AT_TYPE_QUERY;
    } else if (*end == '=') {
        cmd->type = AT_TYPE_SET;
        strncpy(cmd->param, end + 1, sizeof(cmd->param) - 1);
    } else {
        cmd->type = AT_TYPE_EXECUTE;
    }
    return true;
}

/**
 * @brief Parse exactly `count` comma-separated finite numbers
 *
 * "abc", "1.5x", "nan", "inf", a missing value or a wrong count are all
 * invalid: no text silently becomes 0.
 */
static AT_Result_t at_parse_numbers(const char *text, float *values, uint8_t count) {
    const char *p = text;

    for (uint8_t i = 0; i < count; i++) {
        char *end;
        values[i] = strtof(p, &end);
        if (end == p || !isfinite(values[i])) {
            return AT_ERROR_INVALID_PARAM;
        }
        char expected = (char)((i + 1U < count) ? ',' : '\0');
        if (*end != expected) {
            return AT_ERROR_INVALID_PARAM;
        }
        p = end + 1;
    }
    return AT_OK;
}

static const AT_Command_Def_t *at_find(const char *name) {
    for (size_t t = 0; t < table_count; t++) {
        for (size_t i = 0; i < table_sizes[t]; i++) {
            if (strcmp(tables[t][i].name, name) == 0) {
                return &tables[t][i];
            }
        }
    }
    return NULL;
}

/* ==========================================================================
 * Dispatch
 * ========================================================================== */

static void at_run_query(const AT_Command_Def_t *def) {
    if (def->query == NULL) {
        at_reply_result(AT_ERROR_UNKNOWN_CMD);
        return;
    }

    char value[AT_VALUE_SIZE] = "";
    AT_Result_t result = def->query(value, sizeof(value));
    if (result != AT_OK) {
        at_reply_result(result);
        return;
    }

    char text[AT_RESPONSE_BUFFER_SIZE];
    snprintf(text, sizeof(text), "+%s:%s", def->name, value);
    at_reply(text);
}

static void at_run_set(const AT_Command_Def_t *def, const char *param) {
    if (def->set == NULL || def->params == 0 || def->params > AT_MAX_PARAMS) {
        at_reply_result(AT_ERROR_UNKNOWN_CMD);
        return;
    }

    float values[AT_MAX_PARAMS] = { 0.0f };
    AT_Result_t result = at_parse_numbers(param, values, def->params);
    if (result != AT_OK) {
        at_reply_result(result);
        return;
    }

    for (uint8_t i = 0; i < def->params; i++) {
        bool whole = (values[i] == roundf(values[i]));
        if (values[i] < def->min || values[i] > def->max || (def->integer && !whole)) {
            at_reply_result(AT_ERROR_RANGE);
            return;
        }
    }

    at_reply_result(def->set(values));
}

static void at_run_exec(const AT_Command_Def_t *def) {
    at_reply_result(def->exec ? def->exec() : AT_ERROR_UNKNOWN_CMD);
}

void at_cmd_process(UART_Port_t port, const char *line, uint16_t length) {
    reply_port = port;

    if (line == NULL || length == 0) {
        return;
    }

    char buffer[UART_RX_LINE_SIZE];
    if (length >= sizeof(buffer)) {
        length = sizeof(buffer) - 1;
    }
    memcpy(buffer, line, length);
    buffer[length] = '\0';
    str_to_upper(buffer);

    char *start = buffer;
    while (*start == ' ' || *start == '\t') {
        start++;
    }

    if (strncmp(start, "AT", 2) != 0) {
        at_reply("ERROR:Invalid command (must start with AT)");
        return;
    }

    AT_Command_t cmd;
    if (!at_parse_command(start, &cmd)) {
        at_reply("ERROR:Invalid syntax (use AT+CMD?, AT+CMD=val, or AT+CMD)");
        return;
    }

    if (cmd.type == AT_TYPE_TEST) {
        at_reply_result(AT_OK);
        return;
    }

    const AT_Command_Def_t *def = at_find(cmd.name);
    if (def == NULL) {
        at_reply_result(AT_ERROR_UNKNOWN_CMD);
        return;
    }

    switch (cmd.type) {
        case AT_TYPE_QUERY:
            at_run_query(def);
            break;
        case AT_TYPE_SET:
            at_run_set(def, cmd.param);
            break;
        case AT_TYPE_EXECUTE:
            at_run_exec(def);
            break;
        default:
            break;
    }
}

/* ==========================================================================
 * The parser's own commands
 * ========================================================================== */

static AT_Result_t query_version(char *value, size_t size) {
    snprintf(value, size, "%s", FIRMWARE_VERSION);
    return AT_OK;
}

static AT_Result_t exec_reset(void) {
    at_reply_result(AT_OK);
    vTaskDelay(pdMS_TO_TICKS(100));  /* Let the reply leave the UART */
    scb_reset_system();
    return AT_OK;
}

#if AT_CMD_HELP
/** List every registered command, with its syntax, from the tables themselves */
static AT_Result_t exec_help(void) {
    (void)uart_puts(reply_port, "+HELP:\r\n");

    for (size_t t = 0; t < table_count; t++) {
        for (size_t i = 0; i < table_sizes[t]; i++) {
            const AT_Command_Def_t *def = &tables[t][i];
            char syntax[40];
            char line[AT_RESPONSE_BUFFER_SIZE];

            snprintf(syntax, sizeof(syntax), "AT+%s%s%s%s", def->name,
                     def->query ? "?" : "",
                     (def->query && def->set) ? "/" : "",
                     def->set ? ((def->params > 1) ? "=a,b" : "=n") : "");
            snprintf(line, sizeof(line), "  %-20s %s\r\n", syntax, def->help ? def->help : "");
            (void)uart_puts(reply_port, line);
        }
    }
    return AT_OK;
}
#endif // AT_CMD_HELP

static const AT_Command_Def_t system_commands[] = {
    { .name = "VERSION", .query = query_version, .help = AT_HELP("Firmware version") },
    { .name = "RESET",   .exec = exec_reset,     .help = AT_HELP("Reset the MCU") },
#if AT_CMD_HELP
    { .name = "HELP",    .exec = exec_help,      .help = AT_HELP("This list") },
#endif // AT_CMD_HELP
};

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void at_cmd_register(const AT_Command_Def_t *table, size_t count) {
    configASSERT(table_count < AT_MAX_TABLES);

    /* Publish the entry before the count, so a concurrent lookup never sees a hole */
    tables[table_count] = table;
    table_sizes[table_count] = count;
    table_count++;
}

void at_cmd_init(void) {
    at_cmd_register(system_commands, sizeof(system_commands) / sizeof(system_commands[0]));
    uart_set_rx_callback(at_cmd_process);
}
