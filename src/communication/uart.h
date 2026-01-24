/**
 * @file uart.h
 * @brief UART communication driver for STM32F103
 * 
 * Provides asynchronous UART transmission using FreeRTOS queues.
 * Uses USART2 on PA2 (TX) and PA3 (RX).
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>
#include <queue.h>

/* ==========================================================================
 * Public Variables
 * ========================================================================== */

/** UART transmit queue handle */
extern QueueHandle_t uart_txq;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize UART peripheral
 * 
 * Configures USART2 with the following settings:
 * - Baud rate: defined in config.h (default 921600)
 * - 8 data bits, no parity, 1 stop bit
 * - TX only mode
 */
void uart_peripheral_setup(void);

/**
 * @brief Send a null-terminated string via UART
 * 
 * Non-blocking function that queues the string for transmission.
 * The string is copied character by character to the TX queue.
 * 
 * @param s Pointer to null-terminated string
 */
void uart_puts(const char *s);

/**
 * @brief UART transmit task
 * 
 * FreeRTOS task that handles asynchronous UART transmission.
 * Waits for data in the TX queue and transmits it.
 * 
 * @param args Task arguments (unused)
 */
void uart_task(void *args);

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
