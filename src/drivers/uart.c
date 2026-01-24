/**
 * @file uart.c
 * @brief UART communication driver implementation
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "config.h"
#include "drivers/uart.h"
#include "log/log.h"

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

QueueHandle_t uart_txq;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void uart_peripheral_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    /* UART TX on PA2 (GPIO_USART2_TX) */
    gpio_set_mode(GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART2_TX);

    usart_set_baudrate(USART2, UART_BAUDRATE);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);

    /* Create TX queue */
    uart_txq = xQueueCreate(UART_TX_QUEUE_SIZE, sizeof(char));

    log_message(INFO, UART_BUS, "Initialized UART");
}

// -----------------------------------------------------------------------------

void uart_puts(const char *s) {
    for (; *s; ++s) {
        /* Block when queue is full */
        xQueueSend(uart_txq, s, portMAX_DELAY); 
    }
}

void uart_task(void *args) {
    (void)args;
    char ch;

    for (;;) {
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            while (!usart_get_flag(USART2, USART_SR_TXE)) {
                taskYIELD();
            }
            usart_send(USART2, ch);
        }
    }
}