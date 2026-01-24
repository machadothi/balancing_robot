/**
 * @file app_init.c
 * @brief Application initialization implementation
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "config.h"
#include "app/app_init.h"
#include "drivers/uart.h"
#include "cmd/at_cmd.h"
#include "imu/mpu6050.h"
#include "led/led.h"
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

void app_hardware_init(void) {
    /* Configure system clock: 72MHz from 8MHz HSE crystal */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Initialize hardware peripherals */
    led_init();
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
}

void app_tasks_init(void) {
    /* LED heartbeat task */
    xTaskCreate(led_task, TASK_NAME_LED, TASK_STACK_LED, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    
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
}
