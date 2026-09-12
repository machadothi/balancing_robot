/**
 * @file uart.c
 * @brief UART Communication Driver Implementation
 * 
 * Full-duplex UART with interrupt-driven RX and queue-based TX.
 * 
 * RX Flow:
 *   1. Character received -> console UART RX interrupt fires
 *   2. ISR stores character in ring buffer
 *   3. On CR/LF, complete line is queued to uart_rxq
 *   4. uart_rx_task processes lines and invokes callback
 * 
 * TX Flow:
 *   1. Application calls uart_puts/uart_printf
 *   2. Characters queued to uart_txq
 *   3. uart_tx_task sends characters via USART
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "config.h"
#include "board_config.h"
#include "drivers/gpio_compat.h"
#include "drivers/uart.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

#if UART_PRINTF_ENABLED
/** Printf buffer size */
#define UART_PRINTF_BUFFER_SIZE     256
#endif

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** TX queue */
QueueHandle_t uart_txq = NULL;

/** RX line queue */
QueueHandle_t uart_rxq = NULL;

/** RX callback function */
static UART_RxCallback_t rx_callback = NULL;

/** RX line buffer (used in ISR) */
static volatile char rx_line_buffer[UART_RX_LINE_SIZE];
static volatile uint16_t rx_line_pos = 0;

/** TX mutex for thread-safe printf */
static SemaphoreHandle_t tx_mutex = NULL;

/* ==========================================================================
 * Initialization
 * ========================================================================== */

void uart_init(void) {
    /* Enable clocks */
    rcc_periph_clock_enable(BOARD_UART_PORT_RCC);
    rcc_periph_clock_enable(BOARD_UART_RCC);
#if defined(STM32F1)
    rcc_periph_clock_enable(RCC_AFIO);
#endif // defined(STM32F1)

    /* Configure TX and RX pins */
    gpio_compat_af_output(BOARD_UART_PORT, BOARD_UART_TX_PIN, BOARD_UART_AF, false);
    gpio_compat_af_input(BOARD_UART_PORT, BOARD_UART_RX_PIN, BOARD_UART_AF);

    /* Configure USART */
    usart_set_baudrate(BOARD_UART, UART_BAUDRATE);
    usart_set_databits(BOARD_UART, 8);
    usart_set_stopbits(BOARD_UART, USART_STOPBITS_1);
    usart_set_parity(BOARD_UART, USART_PARITY_NONE);
    usart_set_flow_control(BOARD_UART, USART_FLOWCONTROL_NONE);
    usart_set_mode(BOARD_UART, USART_MODE_TX_RX);

    /* Create queues BEFORE enabling interrupts */
    uart_txq = xQueueCreate(UART_TX_QUEUE_SIZE, sizeof(char));
    uart_rxq = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(UART_Line_t));

    /* Create TX mutex */
    tx_mutex = xSemaphoreCreateBinary();
    xSemaphoreGive(tx_mutex);  /* Start in unlocked state */

    /* Enable USART first (needed for TX) */
    usart_enable(BOARD_UART);

    /* Enable RX interrupt AFTER queues are created */
    usart_enable_rx_interrupt(BOARD_UART);
    nvic_set_priority(BOARD_UART_IRQ, 0xC0);  /* Lower priority than DMA */
    nvic_enable_irq(BOARD_UART_IRQ);
}

void uart_peripheral_setup(void) {
    /* Legacy function - now calls full init */
    uart_init();
}

/* ==========================================================================
 * Transmit Functions
 * ========================================================================== */

UART_Status_t uart_putc(char ch) {
    if (uart_txq == NULL) {
        return UART_NOT_INITIALIZED;
    }
    
    if (xQueueSend(uart_txq, &ch, pdMS_TO_TICKS(100)) != pdPASS) {
        return UART_TIMEOUT;
    }
    
    return UART_OK;
}

UART_Status_t uart_puts(const char *s) {
    if (uart_txq == NULL || s == NULL) {
        return UART_NOT_INITIALIZED;
    }
    
    while (*s) {
        xQueueSend(uart_txq, s, portMAX_DELAY);
        s++;
    }
    
    return UART_OK;
}

UART_Status_t uart_write(const uint8_t *data, size_t len) {
    if (uart_txq == NULL || data == NULL) {
        return UART_NOT_INITIALIZED;
    }
    
    for (size_t i = 0; i < len; i++) {
        xQueueSend(uart_txq, &data[i], portMAX_DELAY);
    }
    
    return UART_OK;
}

int uart_printf(const char *fmt, ...) {
#if UART_PRINTF_ENABLED
    if (uart_txq == NULL || tx_mutex == NULL) {
        return -1;
    }
    
    static char buffer[UART_PRINTF_BUFFER_SIZE];
    
    /* Take mutex for thread-safe access to buffer */
    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(100)) != pdPASS) {
        return -1;
    }
    
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    
    if (len > 0) {
        uart_puts(buffer);
    }
    
    xSemaphoreGive(tx_mutex);
    
    return len;
#else
    (void)fmt;
    return 0;
#endif
}

void uart_println(const char *s) {
    if (s != NULL) {
        uart_puts(s);
    }
    uart_puts("\r\n");
}

/* ==========================================================================
 * Receive Functions
 * ========================================================================== */

bool uart_rx_available(void) {
    if (uart_rxq == NULL) {
        return false;
    }
    return uxQueueMessagesWaiting(uart_rxq) > 0;
}

UART_Status_t uart_getline(char *line, size_t max_len, uint32_t timeout_ms) {
    if (uart_rxq == NULL || line == NULL) {
        return UART_NOT_INITIALIZED;
    }
    
    UART_Line_t rx_line;
    TickType_t timeout = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    
    if (xQueueReceive(uart_rxq, &rx_line, timeout) != pdPASS) {
        return UART_TIMEOUT;
    }
    
    /* Copy to user buffer */
    size_t copy_len = (rx_line.length < max_len - 1) ? rx_line.length : max_len - 1;
    memcpy(line, rx_line.data, copy_len);
    line[copy_len] = '\0';
    
    return UART_OK;
}

void uart_set_rx_callback(UART_RxCallback_t callback) {
    rx_callback = callback;
}

/* ==========================================================================
 * Interrupt Handler
 * ========================================================================== */

void uart_rx_isr(void) {
    BaseType_t higher_priority_woken = pdFALSE;
    
    /* Check if data is available */
    if (usart_get_flag(BOARD_UART, USART_SR_RXNE)) {
        char ch = (char)usart_recv(BOARD_UART);
        
#if UART_ECHO_ENABLED
        /* Echo received character */
        while (!usart_get_flag(BOARD_UART, USART_SR_TXE));
        usart_send(BOARD_UART, ch);
#endif
        
        /* Handle line terminator */
        if (ch == '\r' || ch == '\n') {
#if UART_ECHO_ENABLED
            /* Echo newline */
            while (!usart_get_flag(BOARD_UART, USART_SR_TXE));
            usart_send(BOARD_UART, '\n');
#endif
            
            if (rx_line_pos > 0) {
                /* Complete line received - queue it */
                UART_Line_t line;
                line.length = rx_line_pos;
                memcpy(line.data, (const char *)rx_line_buffer, rx_line_pos);
                line.data[rx_line_pos] = '\0';
                
                xQueueSendFromISR(uart_rxq, &line, &higher_priority_woken);
                rx_line_pos = 0;
            }
        }
        /* Handle backspace */
        else if (ch == '\b' || ch == 0x7F) {
            if (rx_line_pos > 0) {
                rx_line_pos--;
            }
        }
        /* Normal character */
        else if (rx_line_pos < UART_RX_LINE_SIZE - 1) {
            rx_line_buffer[rx_line_pos++] = ch;
        }
    }
    
    /* Clear overrun error if set */
    if (usart_get_flag(BOARD_UART, USART_SR_ORE)) {
        (void)usart_recv(BOARD_UART);  /* Clear by reading */
    }
    
    portYIELD_FROM_ISR(higher_priority_woken);
}

/* ==========================================================================
 * Tasks
 * ========================================================================== */

void uart_tx_task(void *args) {
    (void)args;
    char ch;

    for (;;) {
        if (xQueueReceive(uart_txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
            while (!usart_get_flag(BOARD_UART, USART_SR_TXE)) {
                taskYIELD();
            }
            usart_send(BOARD_UART, ch);
        }
    }
}

void uart_rx_task(void *args) {
    (void)args;
    UART_Line_t line;

    for (;;) {
        /* Wait for a complete line */
        if (xQueueReceive(uart_rxq, &line, portMAX_DELAY) == pdPASS) {
            /* Invoke callback if registered */
            if (rx_callback != NULL) {
                rx_callback(line.data, line.length);
            }
        }
    }
}

void uart_task(void *args) {
    /* Legacy task - now just TX */
    uart_tx_task(args);
}
