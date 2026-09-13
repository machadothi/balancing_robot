/**
 * @file fault_handlers.c
 * @brief Fault and error handlers implementation
 * 
 * Provides handlers for:
 * - Hard faults (CPU exceptions)
 * - Stack overflows (FreeRTOS)
 * - Malloc failures (FreeRTOS heap)
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <stdio.h>

#include <libopencm3/stm32/usart.h>

#include "config.h"
#include "board/board.h"
#include "board_config.h"
#include "fault/fault_handlers.h"
#include "motor/motor.h"

/* ==========================================================================
 * Private Helpers
 * ========================================================================== */

#if FAULT_HANDLERS_VERBOSE
/**
 * @brief Send string via USART (blocking, no RTOS)
 */
static void fault_puts(const char *s) {
    while (*s) {
        usart_send_blocking(BOARD_UART, *s++);
    }
}
#endif // FAULT_HANDLERS_VERBOSE

/**
 * @brief Cut motor drive before anything else
 *
 * PWM timers keep running after the CPU stops, so a halted robot would
 * otherwise keep driving at its last duty cycle.
 */
static void fault_stop_motors(void) {
#if !APP_BLINK_ONLY
    motor_emergency_stop();
#endif // !APP_BLINK_ONLY
}

/**
 * @brief Blink LED in infinite loop (fault indicator)
 * @param delay Delay loop count (smaller = faster blink)
 */
static void fault_blink_forever(volatile int delay) {
    for (;;) {
        board_led_toggle();
        for (volatile int i = 0; i < delay; i++);
    }
}

/* ==========================================================================
 * Fault Handlers
 * ========================================================================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    fault_stop_motors();
    
#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! STACK OVERFLOW: ");
    if (pcTaskName) {
        fault_puts(pcTaskName);
    }
    fault_puts("\r\n");
#else
    (void)pcTaskName;
#endif // FAULT_HANDLERS_VERBOSE
    
    /* Fast blink: stack overflow */
    fault_blink_forever(100000);
}

void hard_fault_handler(void) {
    fault_stop_motors();

#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! HARD FAULT !!!\r\n");
#endif // FAULT_HANDLERS_VERBOSE
    
    /* Very fast blink: hard fault */
    fault_blink_forever(50000);
}

void vApplicationMallocFailedHook(void) {
    fault_stop_motors();

#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! MALLOC FAILED !!!\r\n");
#endif // FAULT_HANDLERS_VERBOSE
    
    /* Medium blink: malloc failure */
    fault_blink_forever(200000);
}

void vAssertCalled(const char *file, int line) {
    taskDISABLE_INTERRUPTS();
    fault_stop_motors();

#if FAULT_HANDLERS_VERBOSE
    char line_text[12];
    snprintf(line_text, sizeof(line_text), "%d", line);
    fault_puts("\r\n!!! ASSERT: ");
    fault_puts(file);
    fault_puts(":");
    fault_puts(line_text);
    fault_puts("\r\n");
#else
    (void)file;
    (void)line;
#endif // FAULT_HANDLERS_VERBOSE

    /* Fastest blink: failed assertion */
    fault_blink_forever(25000);
}
