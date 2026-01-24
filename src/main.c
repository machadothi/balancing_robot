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

#include "app/app_init.h"
#include "fault/fault_handlers.h"

/* ==========================================================================
 * Main Entry Point
 * ========================================================================== */

int main(void) {
    /* Initialize all hardware and subsystems */
    app_hardware_init();
    
    /* Create FreeRTOS tasks */
    app_tasks_init();
    
    /* Start scheduler - this should never return */
    vTaskStartScheduler();
    
    /* Should never reach here */
    for (;;) {}

    return 0;
}

