/**
 * @file uart.h
 * @brief Multi-port UART driver: interrupt-driven ring buffer TX, line-based RX
 *
 * Ports follow the features: UART_PORT_USB exists with CONSOLE_USB,
 * UART_PORT_BT with CONSOLE_BT. Their pins come from board_config.h.
 *
 * Writes are atomic: a string is queued whole or not at all, so lines written
 * by different tasks (telemetry, AT replies) never interleave mid-line.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <FreeRTOS.h>

#include "config.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum received line length (including null terminator) */
#define UART_RX_LINE_SIZE       128

/** Complete lines waiting for uart_rx_task, all ports together */
#define UART_RX_QUEUE_SIZE      8

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

typedef enum {
#if CONSOLE_USB
    UART_PORT_USB,          /**< Console: telemetry and full AT access */
#endif // CONSOLE_USB
#if CONSOLE_BT
    UART_PORT_BT,           /**< Bluetooth module: AT commands only */
#endif // CONSOLE_BT
    UART_PORT_COUNT
} UART_Port_t;

typedef enum {
    UART_OK = 0,            /**< Queued for transmission */
    UART_ERROR,             /**< Invalid port or argument */
    UART_TIMEOUT,           /**< No room in the TX buffer before the timeout */
    UART_OVERFLOW,          /**< Did not fit (non-blocking write, or longer than the buffer) */
} UART_Status_t;

/**
 * @brief Called from uart_rx_task for every complete line (CR or LF terminated)
 */
typedef void (*UART_RxCallback_t)(UART_Port_t port, const char *line, uint16_t length);

/* ==========================================================================
 * Functions
 * ========================================================================== */

/** Configure all board ports and create the RX line queue */
void uart_init(void);

/**
 * @brief Queue bytes for transmission as one atomic block
 *
 * Waits for buffer space up to `timeout` ticks (portMAX_DELAY = forever,
 * 0 = never wait). Never call from an ISR.
 */
UART_Status_t uart_write(UART_Port_t port, const char *data, size_t len, TickType_t timeout);

/** Queue a string, waiting up to one second for buffer space */
UART_Status_t uart_puts(UART_Port_t port, const char *s);

/** Queue a string only if it fits right now */
UART_Status_t uart_try_puts(UART_Port_t port, const char *s);

/** Register the handler for received lines (NULL to disable) */
void uart_set_rx_callback(UART_RxCallback_t callback);

/** Echoes (on ports configured to) and dispatches received lines */
void uart_rx_task(void *args);

/** Interrupt handler body, called from the board's USART ISRs in interrupts.c */
void uart_isr(UART_Port_t port);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // UART_H
