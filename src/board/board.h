/**
 * @file board.h
 * @brief Board support interface, implemented once per board (board_<board>.c)
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/** Configure the system clock tree (HSE + PLL) */
void board_clock_init(void);

/** Configure the status LED GPIO(s), LED(s) off */
void board_led_init(void);

/** Toggle the heartbeat LED */
void board_led_toggle(void);

/**
 * @brief Start the independent watchdog
 *
 * Cannot be stopped once started. Frozen while a debugger halts the core.
 *
 * @param timeout_ms  Reset if not refreshed within this time
 */
void board_watchdog_start(uint32_t timeout_ms);

/** Refresh the independent watchdog */
void board_watchdog_refresh(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // BOARD_H
