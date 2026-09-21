/**
 * @file gpio_compat.h
 * @brief Pin configuration helpers hiding the STM32F1 / STM32F4 GPIO API difference
 *
 * The af argument selects the alternate function on STM32F4 and is ignored on
 * STM32F1, where peripheral pins are fixed (or chosen through AFIO remap).
 */

#ifndef GPIO_COMPAT_H
#define GPIO_COMPAT_H

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/gpio.h>

static inline void gpio_compat_af_output(uint32_t port, uint16_t pins,
                                         uint8_t af, bool open_drain) {
#if defined(STM32F1)
    (void)af;
    gpio_set_mode(port, GPIO_MODE_OUTPUT_50_MHZ,
                  open_drain ? GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN
                             : GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  pins);
#else
    gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
    gpio_set_output_options(port, open_drain ? GPIO_OTYPE_OD : GPIO_OTYPE_PP,
                            GPIO_OSPEED_50MHZ, pins);
    gpio_set_af(port, af, pins);
#endif // defined(STM32F1)
}

static inline void gpio_compat_af_input(uint32_t port, uint16_t pins, uint8_t af) {
#if defined(STM32F1)
    (void)af;
    gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pins);
#else
    gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pins);
    gpio_set_af(port, af, pins);
#endif // defined(STM32F1)
}

static inline void gpio_compat_output(uint32_t port, uint16_t pins, bool open_drain) {
#if defined(STM32F1)
    gpio_set_mode(port, GPIO_MODE_OUTPUT_2_MHZ,
                  open_drain ? GPIO_CNF_OUTPUT_OPENDRAIN : GPIO_CNF_OUTPUT_PUSHPULL,
                  pins);
#else
    gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    gpio_set_output_options(port, open_drain ? GPIO_OTYPE_OD : GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ, pins);
#endif // defined(STM32F1)
}

static inline void gpio_compat_input_pullup(uint32_t port, uint16_t pins) {
#if defined(STM32F1)
    gpio_set_mode(port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, pins);
    gpio_set(port, pins);   /* ODR high selects the pull-up on the F1 */
#else
    gpio_mode_setup(port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, pins);
#endif // defined(STM32F1)
}

#endif // GPIO_COMPAT_H
