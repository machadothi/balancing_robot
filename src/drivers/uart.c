/**
 * @file uart.c
 * @brief Multi-port UART driver implementation
 *
 * TX: writers copy into a per-port ring buffer and enable the TXE interrupt;
 *     the ISR sends one byte per interrupt and disables TXE when empty.
 *     Writers are serialised by suspending the scheduler during the copy
 *     (interrupts stay enabled); only the head update and the TXE enable are
 *     done in a critical section, so the ISR never misses new data.
 *
 * RX: the ISR assembles lines per port and posts complete ones, tagged with
 *     their port, to a single queue served by uart_rx_task.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "config.h"
#include "board_config.h"
#include "drivers/gpio_compat.h"
#include "drivers/uart.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

#define UART_PUTS_TIMEOUT_MS    1000

/** Below the I2C/DMA interrupts, still allowed to call FreeRTOS FromISR APIs */
#define UART_IRQ_PRIORITY       0xC0

typedef struct {
    uint32_t usart;
    enum rcc_periph_clken usart_rcc;
    uint8_t irq;
    uint32_t gpio;
    enum rcc_periph_clken gpio_rcc;
    uint16_t tx_pin;
    uint16_t rx_pin;
    uint8_t af;
    uint32_t baudrate;
    bool echo;                  /**< Echo each received line before dispatching it */
    char *tx_buf;
    uint16_t tx_size;           /**< Power of two */
} UART_PortConfig_t;

typedef struct {
    volatile uint16_t tx_head;  /**< Written by tasks */
    volatile uint16_t tx_tail;  /**< Written by the ISR */
    char rx_line[UART_RX_LINE_SIZE];
    uint16_t rx_pos;
} UART_PortState_t;

typedef struct {
    UART_Port_t port;
    uint16_t length;
    char data[UART_RX_LINE_SIZE];
} UART_Line_t;

#if CONSOLE_USB
static char usb_tx_buf[UART_USB_TX_BUFFER_SIZE];
#endif // CONSOLE_USB
#if CONSOLE_BT
static char bt_tx_buf[UART_BT_TX_BUFFER_SIZE];
#endif // CONSOLE_BT

static const UART_PortConfig_t port_config[UART_PORT_COUNT] = {
#if CONSOLE_USB
    [UART_PORT_USB] = {
        BOARD_UART, BOARD_UART_RCC, BOARD_UART_IRQ,
        BOARD_UART_PORT, BOARD_UART_PORT_RCC, BOARD_UART_TX_PIN, BOARD_UART_RX_PIN,
        BOARD_UART_AF, UART_BAUDRATE, CONSOLE_ECHO,
        usb_tx_buf, sizeof(usb_tx_buf),
    },
#endif // CONSOLE_USB
#if CONSOLE_BT
    [UART_PORT_BT] = {
        BOARD_BT_UART, BOARD_BT_UART_RCC, BOARD_BT_UART_IRQ,
        BOARD_BT_UART_PORT, BOARD_BT_UART_PORT_RCC, BOARD_BT_UART_TX_PIN, BOARD_BT_UART_RX_PIN,
        BOARD_BT_UART_AF, BT_BAUDRATE, false,
        bt_tx_buf, sizeof(bt_tx_buf),
    },
#endif // CONSOLE_BT
};

static UART_PortState_t port_state[UART_PORT_COUNT];

static QueueHandle_t uart_rxq = NULL;

static UART_RxCallback_t rx_callback = NULL;

/* ==========================================================================
 * Initialization
 * ========================================================================== */

void uart_init(void) {
    /* Create the queue before any RX interrupt can post to it */
    uart_rxq = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(UART_Line_t));

    for (int p = 0; p < UART_PORT_COUNT; p++) {
        const UART_PortConfig_t *cfg = &port_config[p];

        configASSERT((cfg->tx_size & (cfg->tx_size - 1U)) == 0U);

        rcc_periph_clock_enable(cfg->gpio_rcc);
        rcc_periph_clock_enable(cfg->usart_rcc);
#if defined(STM32F1)
        rcc_periph_clock_enable(RCC_AFIO);
#endif // defined(STM32F1)

        gpio_compat_af_output(cfg->gpio, cfg->tx_pin, cfg->af, false);
        gpio_compat_af_input(cfg->gpio, cfg->rx_pin, cfg->af);

        usart_set_baudrate(cfg->usart, cfg->baudrate);
        usart_set_databits(cfg->usart, 8);
        usart_set_stopbits(cfg->usart, USART_STOPBITS_1);
        usart_set_parity(cfg->usart, USART_PARITY_NONE);
        usart_set_flow_control(cfg->usart, USART_FLOWCONTROL_NONE);
        usart_set_mode(cfg->usart, USART_MODE_TX_RX);
        usart_enable(cfg->usart);

        usart_enable_rx_interrupt(cfg->usart);
        nvic_set_priority(cfg->irq, UART_IRQ_PRIORITY);
        nvic_enable_irq(cfg->irq);
    }
}

/* ==========================================================================
 * Transmit
 * ========================================================================== */

UART_Status_t uart_write(UART_Port_t port, const char *data, size_t len, TickType_t timeout) {
    if (port >= UART_PORT_COUNT || data == NULL) {
        return UART_ERROR;
    }

    const UART_PortConfig_t *cfg = &port_config[port];
    UART_PortState_t *st = &port_state[port];
    const uint16_t mask = (uint16_t)(cfg->tx_size - 1U);

    /* One slot always stays empty to tell a full buffer from an empty one */
    if (len > mask) {
        return UART_OVERFLOW;
    }

    const TickType_t start = xTaskGetTickCount();

    for (;;) {
        bool queued = false;

        vTaskSuspendAll();
        uint16_t head = st->tx_head;
        uint16_t used = (uint16_t)(head - st->tx_tail) & mask;

        if ((size_t)(mask - used) >= len) {
            for (size_t i = 0; i < len; i++) {
                cfg->tx_buf[(head + i) & mask] = data[i];
            }
            taskENTER_CRITICAL();
            st->tx_head = (uint16_t)((head + len) & mask);
            usart_enable_tx_interrupt(cfg->usart);
            taskEXIT_CRITICAL();
            queued = true;
        }
        (void)xTaskResumeAll();

        if (queued) {
            return UART_OK;
        }
        if (timeout == 0) {
            return UART_OVERFLOW;
        }
        if (timeout != portMAX_DELAY && (xTaskGetTickCount() - start) >= timeout) {
            return UART_TIMEOUT;
        }
        vTaskDelay(1);
    }
}

UART_Status_t uart_puts(UART_Port_t port, const char *s) {
    if (s == NULL) {
        return UART_ERROR;
    }
    return uart_write(port, s, strlen(s), pdMS_TO_TICKS(UART_PUTS_TIMEOUT_MS));
}

UART_Status_t uart_try_puts(UART_Port_t port, const char *s) {
    if (s == NULL) {
        return UART_ERROR;
    }
    return uart_write(port, s, strlen(s), 0);
}

/* ==========================================================================
 * Receive
 * ========================================================================== */

void uart_set_rx_callback(UART_RxCallback_t callback) {
    rx_callback = callback;
}

void uart_rx_task(void *args) {
    (void)args;
    UART_Line_t line;
    char echo[UART_RX_LINE_SIZE + 2];

    for (;;) {
        if (xQueueReceive(uart_rxq, &line, portMAX_DELAY) != pdPASS) {
            continue;
        }

        /* Echoed as a whole line, not per character from the ISR: a lone
         * character could otherwise land inside a telemetry line */
        if (port_config[line.port].echo) {
            memcpy(echo, line.data, line.length);
            memcpy(echo + line.length, "\r\n", 2);
            (void)uart_write(line.port, echo, line.length + 2U,
                             pdMS_TO_TICKS(UART_PUTS_TIMEOUT_MS));
        }

        if (rx_callback != NULL) {
            rx_callback(line.port, line.data, line.length);
        }
    }
}

/* ==========================================================================
 * Interrupt Handler
 * ========================================================================== */

void uart_isr(UART_Port_t port) {
    const UART_PortConfig_t *cfg = &port_config[port];
    UART_PortState_t *st = &port_state[port];
    const uint32_t usart = cfg->usart;
    BaseType_t higher_priority_woken = pdFALSE;

    /* RXNE is also set on overrun; reading the data register clears both */
    if (usart_get_flag(usart, USART_SR_RXNE)) {
        char ch = (char)usart_recv(usart);

        if (ch == '\r' || ch == '\n') {
            if (st->rx_pos > 0) {
                UART_Line_t line = { .port = port, .length = st->rx_pos };
                memcpy(line.data, st->rx_line, st->rx_pos);
                line.data[st->rx_pos] = '\0';
                (void)xQueueSendFromISR(uart_rxq, &line, &higher_priority_woken);
                st->rx_pos = 0;
            }
        } else if (ch == '\b' || ch == 0x7F) {
            if (st->rx_pos > 0) {
                st->rx_pos--;
            }
        } else if (st->rx_pos < UART_RX_LINE_SIZE - 1) {
            st->rx_line[st->rx_pos++] = ch;
        }
    }

    if ((USART_CR1(usart) & USART_CR1_TXEIE) && usart_get_flag(usart, USART_SR_TXE)) {
        const uint16_t tail = st->tx_tail;
        if (tail != st->tx_head) {
            usart_send(usart, (uint16_t)(uint8_t)cfg->tx_buf[tail]);
            st->tx_tail = (uint16_t)((tail + 1U) & (cfg->tx_size - 1U));
        } else {
            usart_disable_tx_interrupt(usart);
        }
    }

    portYIELD_FROM_ISR(higher_priority_woken);
}
