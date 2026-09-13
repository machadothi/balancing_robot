/**
 * @file at_cmd.h
 * @brief Generic AT command console
 *
 * The parser knows no application commands. Modules register tables of
 * AT_Command_Def_t; the parser checks syntax, number formats and ranges, calls
 * the handler and formats the reply. The command reference is
 * docs/10-at-commands.md.
 *
 *   AT              Test, replies OK
 *   AT+NAME?        Query    -> +NAME:<value>
 *   AT+NAME=a[,b]   Set      -> OK or ERROR:<code>
 *   AT+NAME         Execute  -> OK or ERROR:<code>
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef AT_CMD_H
#define AT_CMD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "drivers/uart.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/** Reply codes, printed as ERROR:<code> */
typedef enum {
    AT_OK = 0,              /**< Success */
    AT_ERROR,               /**< Known command that failed */
    AT_ERROR_UNKNOWN_CMD,   /**< No such command, or it does not support this form */
    AT_ERROR_INVALID_PARAM, /**< Missing, malformed or non-finite number */
    AT_ERROR_RANGE,         /**< Number out of range */
    AT_ERROR_NOT_READY,     /**< System not ready */
    AT_ERROR_BUSY,          /**< System busy */
} AT_Result_t;

/** Most numbers a set command can take */
#define AT_MAX_PARAMS   2

/** Room a query handler has for its value text */
#define AT_VALUE_SIZE   96

/** Write the value of AT+NAME? into `value` (null-terminated) */
typedef AT_Result_t (*AT_QueryFn_t)(char *value, size_t size);

/** Apply AT+NAME=...; `values` holds `params` numbers, already range-checked */
typedef AT_Result_t (*AT_SetFn_t)(const float *values);

/** Run AT+NAME */
typedef AT_Result_t (*AT_ExecFn_t)(void);

typedef struct {
    const char  *name;      /**< Upper-case name without "AT+" */
    AT_QueryFn_t query;     /**< NULL if AT+NAME? is not supported */
    AT_SetFn_t   set;       /**< NULL if AT+NAME= is not supported */
    AT_ExecFn_t  exec;      /**< NULL if AT+NAME is not supported */
    uint8_t      params;    /**< Comma-separated numbers AT+NAME= takes (1..AT_MAX_PARAMS) */
    bool         integer;   /**< Numbers must be whole */
    float        min;       /**< Inclusive range for every number */
    float        max;
    const char  *help;      /**< One-line description for AT+HELP, written with AT_HELP() */
} AT_Command_Def_t;

/** Help strings only take flash when AT+HELP is built */
#if AT_CMD_HELP
#define AT_HELP(text)   (text)
#else
#define AT_HELP(text)   NULL
#endif // AT_CMD_HELP

/** Register the parser's own commands and the UART line callback */
void at_cmd_init(void);

/**
 * @brief Add a table of commands
 *
 * The table must stay valid forever (normally a static const array).
 * Call from init code or a task's start-up, before commands for it arrive.
 */
void at_cmd_register(const AT_Command_Def_t *table, size_t count);

/** Parse and execute one received line; replies go to `port` */
void at_cmd_process(UART_Port_t port, const char *line, uint16_t length);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // AT_CMD_H
