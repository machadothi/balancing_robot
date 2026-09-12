/**
 * @file led.c
 * @brief LED control module implementation
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include "board/board.h"
#include "led/led.h"
#include "log/log.h"

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void led_init(void) {
    board_led_init();
}

void led_task(void *args) {
    (void)args;

    for (;;) {
        TickType_t last_wake_time = xTaskGetTickCount();

        board_led_toggle();

        log_message(LOG_DEBUG, UART_BUS, "LED heartbeat");

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(250));
    }
}
