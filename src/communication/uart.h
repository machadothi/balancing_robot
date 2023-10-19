#ifndef UART_H_
#define UART_H_

#include <FreeRTOS.h>
#include <queue.h>

typedef enum {
    I2C_Ok = 0,
    I2C_Write,
    I2C_Read
} UART_Fails;

extern QueueHandle_t uart_txq;

void uart_peripheral_setup(void);

void uart_puts(const char *s);

void uart_task(void *args __attribute__((unused)));

#endif // UART_H_