/**
 * @file app_init.c
 * @brief Application initialization implementation
 *
 * The one place that wires modules together: which ones exist depends on the
 * features in app_config.h (prj.conf).
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "app/app_init.h"
#include "board/board.h"
#include "led/led.h"

#if !APP_BLINK_ONLY
#include "imu/imu.h"
#include "robot/robot.h"
#endif // !APP_BLINK_ONLY

#if CONSOLE_ANY
#include "drivers/uart.h"
#include "cmd/at_cmd.h"
#endif // CONSOLE_ANY

#if TELEMETRY
#include "telemetry/telemetry.h"
#endif // TELEMETRY

#if CONSOLE_USB
#include <libopencm3/stm32/usart.h>

#include "board_config.h"
#include "log/log.h"

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Log output wrapper: logs go to the USB console
 */
static void log_uart_send(const char *message) {
    uart_puts(UART_PORT_USB, message);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void app_print_banner(void) {
    const char *banner = "\r\n=== Balancing Robot v1.0 ===\r\nAT Command Ready\r\n> ";
    for (const char *p = banner; *p; p++) {
        usart_send_blocking(BOARD_UART, *p);
    }
}
#endif // CONSOLE_USB

void app_hardware_init(void) {
    board_clock_init();

    /* Initialize hardware peripherals */
    led_init();

#if CONSOLE_ANY
    uart_init();
#endif // CONSOLE_ANY

#if CONSOLE_USB
    /* Send startup banner (blocking, before tasks start) */
    app_print_banner();

    /* Configure logging */
    static LogDriver_t log_driver = {
        .log_level = LOG_INFO,
        .send = log_uart_send
    };
    log_init(&log_driver);
#endif // CONSOLE_USB

    /* Create queues before creating tasks */
#if !APP_BLINK_ONLY
    imu_queue_init();
#endif // !APP_BLINK_ONLY
#if TELEMETRY
    telemetry_init();
#endif // TELEMETRY

#if CONSOLE_ANY
    /* Initialize AT command parser (registers RX callback) */
    at_cmd_init();
#endif // CONSOLE_ANY
}

void app_tasks_init(void) {
    /* LED heartbeat task */
    xTaskCreate(led_task, TASK_NAME_LED, TASK_STACK_LED,
        NULL, TASK_PRIORITY_LED, NULL);

#if CONSOLE_ANY
    /* UART RX task (AT commands from every console port) */
    xTaskCreate(uart_rx_task, TASK_NAME_UART_RX, TASK_STACK_UART_RX,
        NULL, TASK_PRIORITY_IO, NULL);
#endif // CONSOLE_ANY

#if TELEMETRY
    /* Telemetry logger (USB console) */
    xTaskCreate(telemetry_task, TASK_NAME_TELEMETRY, TASK_STACK_TELEMETRY,
        NULL, TASK_PRIORITY_IO, NULL);
#endif // TELEMETRY

#if !APP_BLINK_ONLY
    /* IMU reading task */
    xTaskCreate(imu_task, TASK_NAME_IMU, TASK_STACK_IMU,
        NULL, TASK_PRIORITY_CONTROL, NULL);

    /* Robot control task */
    xTaskCreate(robot_task, TASK_NAME_ROBOT, TASK_STACK_ROBOT,
        NULL, TASK_PRIORITY_CONTROL, NULL);
#endif // !APP_BLINK_ONLY
}
