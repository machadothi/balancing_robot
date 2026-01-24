/**
 * @file led.c
 * @brief LED control module implementation
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "led/led.h"
#include "log/log.h"

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void led_init(void) {
    /* Blue Pill onboard LED (PC13) */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
        GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* RGB LED (PB12=Green, PB13=Blue, PB14=Red) */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, 
        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12 | GPIO13 | GPIO14);
}

void led_task(void *args) {
    (void)args;
    
    for (;;) {
        TickType_t last_wake_time = xTaskGetTickCount();

        gpio_toggle(GPIOC, GPIO13);     /* Toggle onboard LED */
        gpio_set(GPIOB, GPIO12);        /* Green off */
        gpio_set(GPIOB, GPIO13);        /* Blue off */
        gpio_toggle(GPIOB, GPIO14);     /* Toggle red */

        log_message(LOG_DEBUG, UART_BUS, "LED heartbeat");
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(250));
    }
}
