/**
 * @file board_f407.c
 * @brief Hiwonder ROS Robot Control Board (STM32F407VET6) board support
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/dbgmcu.h>

#include "board/board.h"

/* On F4 the IWDG debug freeze lives in DBGMCU_APB1_FZ, which libopencm3 does not name */
#define DBGMCU_APB1_FZ                  MMIO32(DBGMCU_BASE + 0x08)
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP    (1U << 12)

void board_watchdog_start(uint32_t timeout_ms) {
    /* Without this, halting at a breakpoint resets the MCU */
    DBGMCU_APB1_FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    iwdg_set_period_ms(timeout_ms);
    iwdg_start();
}

void board_watchdog_refresh(void) {
    iwdg_reset();
}

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
