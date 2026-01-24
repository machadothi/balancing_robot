/**
 * @file app_init.h
 * @brief Application initialization functions
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * @brief Initialize all hardware peripherals
 * 
 * Configures:
 * - System clock (72MHz from HSE)
 * - LED GPIO
 * - UART (TX/RX with interrupts)
 * - Logging subsystem
 * - IMU queue
 * - AT command parser
 */
void app_hardware_init(void);

/**
 * @brief Create all FreeRTOS tasks
 * 
 * Creates:
 * - LED blink task
 * - UART TX task
 * - UART RX task (AT command processing)
 * - IMU reading task
 * - Robot control task
 */
void app_tasks_init(void);

/**
 * @brief Send startup banner (blocking)
 */
void app_print_banner(void);

#endif /* APP_INIT_H */
