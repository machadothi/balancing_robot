/**
 * @file main.c
 * @brief Self-Balancing Robot - Main Application Entry
 * 
 * This project implements a two-wheeled self-balancing robot using:
 * - STM32F103C8T6 (Blue Pill) microcontroller
 * - MPU6050 IMU for tilt sensing
 * - FreeRTOS for task management
 * - Kalman and Complementary filters for sensor fusion
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
#include "imu/mpu6050.h"
#include "led/led.h"
#include "log/log.h"
#include "robot/robot.h"

#include <libopencm3/stm32/gpio.h>

/* ==========================================================================
 * FreeRTOS Hooks
 * ========================================================================== */

/* Declared in FreeRTOS task.h but needs definition */
extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    
    /* Fast blink LED on stack overflow */
    for (;;) {
        gpio_toggle(GPIOC, GPIO13);
        for (volatile int i = 0; i < 100000; i++);
    }
}

/* ==========================================================================
 * Main Entry Point
 * ========================================================================== */

int main(void) {
    /* Configure system clock: 72MHz from 8MHz HSE crystal */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    /* Initialize hardware */
    led_init();
    uart_peripheral_setup();

    /* Send startup banner (blocking, before tasks start) */
    const char *banner = "\r\n=== Balancing Robot v1.0 ===\r\n";
    for (const char *p = banner; *p; p++) {
        usart_send_blocking(USART2, *p);
    }

    /* Configure logging */
    static LogDriver_t log_driver = {
        .log_level = LOG_INFO,
        .send = uart_puts
    };
    log_init(&log_driver);

    /* Initialize IMU queue before creating tasks */
    imu_queue_init();

    /* Create FreeRTOS tasks */
    xTaskCreate(led_task, TASK_NAME_LED, TASK_STACK_LED, 
        NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_task, TASK_NAME_UART, TASK_STACK_UART, 
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
