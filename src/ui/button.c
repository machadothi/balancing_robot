/**
 * @file button.c
 * @brief User button: arm or stop balancing without a console
 *
 * Press once with the robot lying down: it is armed, and starts balancing as
 * soon as it is lifted upright. Press again to stop. The pin comes from the
 * board (BOARD_BUTTON_ENABLE_*); what arming means is the robot's business.
 */

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "board_config.h"
#include "app/module.h"
#include "drivers/gpio_compat.h"
#include "robot/robot.h"

#define BUTTON_POLL_MS              10

/** A new level must hold this many samples (30 ms) to count: contact bounce */
#define BUTTON_DEBOUNCE_SAMPLES     3

static void button_init(void) {
    rcc_periph_clock_enable(BOARD_BUTTON_ENABLE_PORT_RCC);
    /* The board has pull-ups; the internal one also keeps the pin defined */
    gpio_compat_input_pullup(BOARD_BUTTON_ENABLE_PORT, BOARD_BUTTON_ENABLE_PIN);
}

/** Active low: the button pulls the pin to ground */
static bool button_is_down(void) {
    return gpio_get(BOARD_BUTTON_ENABLE_PORT, BOARD_BUTTON_ENABLE_PIN) == 0;
}

static void button_task(void *args) {
    (void)args;
    bool stable_down = false;
    uint8_t changed_samples = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        bool down = button_is_down();

        if (down == stable_down) {
            changed_samples = 0;
        } else if (++changed_samples >= BUTTON_DEBOUNCE_SAMPLES) {
            stable_down = down;
            changed_samples = 0;
            if (stable_down) {
                robot_toggle_armed();   /* on the press, not the release */
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

APP_MODULE(button_module) = {
    .name = "BUTTON",
    .init = button_init,
    .task = button_task,
    .stack = 128,
    .priority = APP_PRIORITY_BACKGROUND,
};
