/**
 * @file console.c
 * @brief Console module: UART ports, AT parser, start-up banner and log output
 */

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "app/module.h"
#include "cmd/at_cmd.h"
#include "drivers/uart.h"

/** AT parsing and float formatting run in this task (words) */
#define CONSOLE_TASK_STACK      384

#if CONSOLE_USB
#include <libopencm3/stm32/usart.h>

#include "board_config.h"
#include "log/log.h"

/** Logs go to the USB console */
static void log_uart_send(const char *message) {
    uart_puts(UART_PORT_USB, message);
}

/** Blocking: runs before the scheduler, when interrupt-driven TX is not needed yet */
static void print_banner(void) {
    const char *banner = "\r\n=== Balancing Robot v1.0 ===\r\nAT Command Ready\r\n> ";
    for (const char *p = banner; *p; p++) {
        usart_send_blocking(BOARD_UART, *p);
    }
}
#endif // CONSOLE_USB

static void console_init(void) {
    uart_init();

#if CONSOLE_USB
    print_banner();

    static LogDriver_t log_driver = {
        .log_level = LOG_INFO,
        .send = log_uart_send,
    };
    log_init(&log_driver);
#endif // CONSOLE_USB

    at_cmd_init();
}

const App_Module_t console_module = {
    .name = "UART_RX",
    .init = console_init,
    .task = uart_rx_task,
    .stack = CONSOLE_TASK_STACK,
    .priority = APP_PRIORITY_IO,
};
