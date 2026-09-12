/**
 * @file uart.h
 * @brief UART Communication Driver Interface
 * 
 * Full-duplex UART driver with FreeRTOS queue-based TX/RX.
 * Uses USART2: PA2 (TX), PA3 (RX).
 * 
 * Features:
 *   - Asynchronous TX via queue (non-blocking send)
 *   - Interrupt-driven RX with line buffering
 *   - Configurable baud rate and buffer sizes
 *   - Thread-safe for multi-task access
 *   - Printf-style formatted output
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** Maximum line length for RX (including null terminator) */
#ifndef UART_RX_LINE_SIZE
#define UART_RX_LINE_SIZE       128
#endif

/** RX line queue depth (number of complete lines) */
#ifndef UART_RX_QUEUE_SIZE
#define UART_RX_QUEUE_SIZE      8
#endif

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief UART status codes
 */
typedef enum {
    UART_OK = 0,            /**< Operation successful */
    UART_ERROR,             /**< General error */
    UART_TIMEOUT,           /**< Operation timed out */
    UART_OVERFLOW,          /**< Buffer overflow */
    UART_NOT_INITIALIZED,   /**< UART not initialized */
} UART_Status_t;

/**
 * @brief Received line structure
 */
typedef struct {
    char     data[UART_RX_LINE_SIZE];   /**< Line buffer */
    uint16_t length;                     /**< Line length (excluding null) */
} UART_Line_t;

/**
 * @brief UART RX callback function type
 * 
 * Called when a complete line is received (terminated by CR or LF).
 */
typedef void (*UART_RxCallback_t)(const char *line, uint16_t length);

/* ==========================================================================
 * Public Variables
 * ========================================================================== */

/** UART transmit queue handle */
extern QueueHandle_t uart_txq;

/** UART receive line queue handle */
extern QueueHandle_t uart_rxq;

/* ==========================================================================
 * Initialization
 * ========================================================================== */

/**
 * @brief Initialize UART peripheral with full-duplex support
 * 
 * Configures USART2 for both TX and RX with interrupt-driven reception.
 * Creates TX queue for asynchronous transmission and RX queue for
 * line-buffered reception.
 * 
 * Settings:
 *   - Baud rate: UART_BAUDRATE from config.h
 *   - 8N1 (8 data bits, no parity, 1 stop bit)
 *   - RX interrupt enabled
 */
void uart_init(void);

/**
 * @brief Legacy initialization (TX only)
 * @deprecated Use uart_init() for full functionality
 */
void uart_peripheral_setup(void);

/* ==========================================================================
 * Transmit Functions
 * ========================================================================== */

/**
 * @brief Send a single character
 * 
 * @param ch    Character to send
 * @return UART_OK on success, UART_TIMEOUT if queue full
 */
UART_Status_t uart_putc(char ch);

/**
 * @brief Send a null-terminated string
 * 
 * Non-blocking - characters are queued for transmission.
 * 
 * @param s     Pointer to null-terminated string
 * @return UART_OK on success
 */
UART_Status_t uart_puts(const char *s);

/**
 * @brief Queue a string only if it fits entirely, never blocking
 *
 * For periodic output from time-critical tasks: when the TX queue lacks
 * space the string is dropped instead of stalling the caller.
 *
 * @param s     Pointer to null-terminated string
 * @return UART_OK if queued, UART_OVERFLOW if it did not fit
 */
UART_Status_t uart_try_puts(const char *s);

/**
 * @brief Send a buffer of known length
 * 
 * @param data  Pointer to data buffer
 * @param len   Number of bytes to send
 * @return UART_OK on success
 */
UART_Status_t uart_write(const uint8_t *data, size_t len);

/**
 * @brief Printf-style formatted output
 * 
 * Supports standard printf format specifiers.
 * Maximum output length is 256 characters.
 * 
 * @param fmt   Format string
 * @param ...   Variable arguments
 * @return Number of characters sent, or negative on error
 */
int uart_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Send string with CRLF termination
 * 
 * Useful for AT command responses.
 * 
 * @param s     String to send (CRLF added automatically)
 */
void uart_println(const char *s);

/* ==========================================================================
 * Receive Functions
 * ========================================================================== */

/**
 * @brief Check if a complete line is available
 * 
 * @return true if at least one line is in the RX queue
 */
bool uart_rx_available(void);

/**
 * @brief Get a received line (blocking)
 * 
 * Blocks until a complete line is received or timeout expires.
 * Line terminators (CR/LF) are stripped.
 * 
 * @param line      Buffer to store received line
 * @param max_len   Maximum buffer size
 * @param timeout_ms Timeout in milliseconds (0 = no wait, portMAX_DELAY = forever)
 * @return UART_OK on success, UART_TIMEOUT if no data
 */
UART_Status_t uart_getline(char *line, size_t max_len, uint32_t timeout_ms);

/**
 * @brief Register RX callback
 * 
 * Callback is invoked from uart_rx_task when a complete line is received.
 * Use this for command processing instead of polling uart_getline().
 * 
 * @param callback  Function to call on line reception (NULL to disable)
 */
void uart_set_rx_callback(UART_RxCallback_t callback);

/* ==========================================================================
 * Tasks
 * ========================================================================== */

/**
 * @brief UART transmit task
 * 
 * Handles asynchronous transmission from TX queue.
 * 
 * @param args  Task arguments (unused)
 */
void uart_tx_task(void *args);

/**
 * @brief UART receive processing task
 * 
 * Processes complete lines from RX queue and invokes callback.
 * 
 * @param args  Task arguments (unused)
 */
void uart_rx_task(void *args);

/**
 * @brief Legacy task name for compatibility
 */
void uart_task(void *args);

/* ==========================================================================
 * ISR Handler (called from interrupts.c)
 * ========================================================================== */

/**
 * @brief UART RX interrupt handler
 * 
 * Call this from usart2_isr() in interrupts.c
 */
void uart_rx_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
