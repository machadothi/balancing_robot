#ifndef UART_H_
#define UART_H_

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

typedef enum {
    I2C_Ok = 0,
    I2C_Write,
    I2C_Read
} UART_Fails;

extern QueueHandle_t uart_txq;

/**
 * @brief setup the STM32F1 UART peripheral
 * 
 */
void uart_peripheral_setup(void);

/**
 * @brief Puts a string into the UART queue
 * 
 * @param s string content must be ended with '\0'
 */
void uart_puts(const char *s);

/**
 * @brief tasks waits for content being place into the queue and sends it thru
 * UART peripheral.
 * 
 * @param __attribute__ 
 */
void uart_task(void *args __attribute__((unused)));

#endif // UART_H_