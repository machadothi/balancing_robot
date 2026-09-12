/**
 * @file board_f103.c
 * @brief Blue Pill (STM32F103C8T6) board support
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "board/board.h"

void board_clock_init(void) {
    /* 72MHz from 8MHz HSE crystal */
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void board_led_init(void) {
    /* Blue Pill onboard LED (PC13) */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* RGB LED (PB12=Green, PB13=Blue, PB14=Red) */
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12 | GPIO13 | GPIO14);
}

void board_led_toggle(void) {
    gpio_toggle(GPIOC, GPIO13);     /* Toggle onboard LED */
    gpio_set(GPIOB, GPIO12);        /* Green off */
    gpio_set(GPIOB, GPIO13);        /* Blue off */
    gpio_toggle(GPIOB, GPIO14);     /* Toggle red */
}
