/**
 * @file led.h
 * @brief LED control module interface
 * 
 * Provides LED initialization and heartbeat task for system status indication.
 * 
 * Hardware configuration:
 * - PC13: Blue Pill onboard LED (active low)
 * - PB12: External Green LED
 * - PB13: External Blue LED
 * - PB14: External Red LED
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

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
#endif

#endif /* LED_H */
