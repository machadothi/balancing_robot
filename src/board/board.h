/**
 * @file board.h
 * @brief Board support interface, implemented once per board (board_<board>.c)
 */

#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/** Configure the system clock tree (HSE + PLL) */
void board_clock_init(void);

/** Configure the status LED GPIO(s), LED(s) off */
void board_led_init(void);

/** Toggle the heartbeat LED */
void board_led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
