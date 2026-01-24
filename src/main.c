/**
 * @file main.c
 * @brief Self-Balancing Robot - Main Application Entry
 * 
 * This project implements a two-wheeled self-balancing robot using:
 * - STM32F103C8T6 (Blue Pill) microcontroller
 * - MPU6050 IMU for tilt sensing
 * - FreeRTOS for task management
 * - Kalman and Complementary filters for sensor fusion
 * - AT command interface for remote control
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "config.h"
#include "drivers/uart.h"
#include "cmd/at_cmd.h"
#include "imu/mpu6050.h"
#include "led/led.h"
#include "log/log.h"
#include "robot/robot.h"

#include <libopencm3/stm32/gpio.h>

/* ==========================================================================
 * FreeRTOS Hooks & Fault Handlers
 * ========================================================================== */

/* Prototypes for handlers (suppress -Wmissing-prototypes) */
void hard_fault_handler(void);
void vApplicationMallocFailedHook(void);

/* Declared in FreeRTOS task.h but needs definition */
extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    
    /* Print task name that overflowed */
    const char *msg = "\r\n!!! STACK OVERFLOW: ";
    for (const char *p = msg; *p; p++) {
        usart_send_blocking(USART2, *p);
    }
    if (pcTaskName) {
        for (const char *p = pcTaskName; *p; p++) {
            usart_send_blocking(USART2, *p);
        }
    }
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');
    
    /* Fast blink LED on stack overflow */
    for (;;) {
        gpio_toggle(GPIOC, GPIO13);
        for (volatile int i = 0; i < 100000; i++);
    }
}

/**
 * @brief Hard Fault Handler - prints debug info
 */
void hard_fault_handler(void) {
    const char *msg = "\r\n!!! HARD FAULT !!!\r\n";
    for (const char *p = msg; *p; p++) {
        usart_send_blocking(USART2, *p);
    }
    
    /* Very fast blink LED on hard fault */
    for (;;) {
        gpio_toggle(GPIOC, GPIO13);
        for (volatile int i = 0; i < 50000; i++);
    }
}

/**
 * @brief Malloc failed hook
 */
void vApplicationMallocFailedHook(void) {
    const char *msg = "\r\n!!! MALLOC FAILED !!!\r\n";
    for (const char *p = msg; *p; p++) {
        usart_send_blocking(USART2, *p);
    }
    
    /* Medium blink LED on malloc failure */
    for (;;) {
        gpio_toggle(GPIOC, GPIO13);
        for (volatile int i = 0; i < 200000; i++);
    }
}

/**
 * @brief Log output wrapper for uart_puts
 * 
 * Matches LogDriver_t.send signature (void return).
 * 
 * @param message   String to send
 */
static void log_uart_send(const char *message) {
    uart_puts(message);
}

/* ==========================================================================
 * Main Entry Point
 * ========================================================================== */

int main(void) {
    /* Configure system clock: 72MHz from 8MHz HSE crystal */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Initialize hardware */
    led_init();
    uart_init();  /* Full UART init with RX */

    /* Send startup banner (blocking, before tasks start) */
    const char *banner = "\r\n=== Balancing Robot v1.0 ===\r\nAT Command Ready\r\n> ";
    for (const char *p = banner; *p; p++) {
        usart_send_blocking(USART2, *p);
    }

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

    /* Create FreeRTOS tasks */
    xTaskCreate(led_task, TASK_NAME_LED, TASK_STACK_LED, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_tx_task, TASK_NAME_UART, TASK_STACK_UART, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_rx_task, TASK_NAME_UART_RX, TASK_STACK_UART_RX, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(imu_task, TASK_NAME_IMU, TASK_STACK_IMU, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(robot_task, TASK_NAME_ROBOT, TASK_STACK_ROBOT, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    
    /* Start scheduler - this should never return */
    vTaskStartScheduler();
    
    for (;;) {
        /* Should never reach here */
    }

    return 0;
}
