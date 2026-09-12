/**
 * @file board_f407.c
 * @brief Hiwonder ROS Robot Control Board (STM32F407VET6) board support
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "board/board.h"

void board_clock_init(void) {
    /* 168MHz from 8MHz HSE crystal */
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

void board_led_init(void) {
    /* User LED on PE10, active low */
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
    gpio_set(GPIOE, GPIO10);
}

void board_led_toggle(void) {
    gpio_toggle(GPIOE, GPIO10);
}
