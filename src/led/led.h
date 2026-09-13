/**
 * @file led.h
 * @brief LED control module interface
 * 
 * Provides LED initialization and heartbeat task for system status indication.
 * 
 * LED pins are board specific, see src/board/board_<board>.c.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize LED GPIO pins
 * 
 * Configures PC13 and PB12-14 as push-pull outputs.
 */
void led_init(void);

/**
 * @brief LED heartbeat task
 * 
 * FreeRTOS task that blinks LEDs to indicate system is running.
 * Toggles onboard LED and red LED every 250ms.
 * 
 * @param args Task arguments (unused)
 */
void led_task(void *args);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // LED_H
