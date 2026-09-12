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

#include <libopencm3/stm32/usart.h>

#include "config.h"
#include "board/board.h"
#include "fault/fault_handlers.h"

/* ==========================================================================
 * Private Helpers
 * ========================================================================== */

#if FAULT_HANDLERS_VERBOSE
/**
 * @brief Send string via USART (blocking, no RTOS)
 */
static void fault_puts(const char *s) {
    while (*s) {
        usart_send_blocking(USART2, *s++);
    }
}
#endif

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
    
#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! STACK OVERFLOW: ");
    if (pcTaskName) {
        fault_puts(pcTaskName);
    }
    fault_puts("\r\n");
#else
    (void)pcTaskName;
#endif
    
    /* Fast blink: stack overflow */
    fault_blink_forever(100000);
}

void hard_fault_handler(void) {
#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! HARD FAULT !!!\r\n");
#endif
    
    /* Very fast blink: hard fault */
    fault_blink_forever(50000);
}

void vApplicationMallocFailedHook(void) {
#if FAULT_HANDLERS_VERBOSE
    fault_puts("\r\n!!! MALLOC FAILED !!!\r\n");
#endif
    
    /* Medium blink: malloc failure */
    fault_blink_forever(200000);
}
