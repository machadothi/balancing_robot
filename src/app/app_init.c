/**
 * @file app_init.c
 * @brief Application initialization implementation
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
#include <libopencm3/stm32/usart.h>

#include "drivers/uart.h"
#include "cmd/at_cmd.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "robot/robot.h"

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Log output wrapper for uart_puts
 */
static void log_uart_send(const char *message) {
    uart_puts(message);
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void app_print_banner(void) {
    const char *banner = "\r\n=== Balancing Robot v1.0 ===\r\nAT Command Ready\r\n> ";
    for (const char *p = banner; *p; p++) {
        usart_send_blocking(USART2, *p);
    }
}
#endif /* !APP_BLINK_ONLY */

void app_hardware_init(void) {
    board_clock_init();

    /* Initialize hardware peripherals */
    led_init();

#if !APP_BLINK_ONLY
    uart_init();

    /* Send startup banner (blocking, before tasks start) */
    app_print_banner();

    /* Configure logging */
    static LogDriver_t log_driver = {
        .log_level = LOG_INFO,
        .send = log_uart_send
    };
    log_init(&log_driver);

    /* Initialize IMU queue before creating tasks */
    imu_queue_init();

    /* Initialize AT command parser (registers RX callback) */
    at_cmd_init();
#endif
}

void app_tasks_init(void) {
    /* LED heartbeat task */
    xTaskCreate(led_task, TASK_NAME_LED, TASK_STACK_LED,
        NULL, configMAX_PRIORITIES - 1, NULL);

#if !APP_BLINK_ONLY
    /* UART TX task */
    xTaskCreate(uart_tx_task, TASK_NAME_UART, TASK_STACK_UART,
        NULL, configMAX_PRIORITIES - 1, NULL);

    /* UART RX task (processes AT commands) */
    xTaskCreate(uart_rx_task, TASK_NAME_UART_RX, TASK_STACK_UART_RX,
        NULL, configMAX_PRIORITIES - 1, NULL);

    /* IMU reading task */
    xTaskCreate(imu_task, TASK_NAME_IMU, TASK_STACK_IMU,
        NULL, configMAX_PRIORITIES - 1, NULL);

    /* Robot control task */
    xTaskCreate(robot_task, TASK_NAME_ROBOT, TASK_STACK_ROBOT,
        NULL, configMAX_PRIORITIES - 1, NULL);
#endif
}
