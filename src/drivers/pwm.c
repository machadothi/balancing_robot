/**
 * @file pwm.c
 * @brief PWM (Pulse Width Modulation) Driver Implementation
 *
 * Timer-based PWM generation in PWM mode 1 with configurable frequency and
 * resolution. Supported timers: TIM1-4 on STM32F1, TIM1-5 and TIM9-11 on
 * STM32F4.
 *
 * Clock Configuration:
 *   - Timer clock = APB bus clock, doubled when that bus is divided from AHB
 *   - PWM frequency = Timer_Clock / ((Prescaler + 1) * (Period + 1))
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <stddef.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "drivers/gpio_compat.h"
#include "drivers/pwm.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Minimum supported PWM frequency */
#define PWM_MIN_FREQUENCY_HZ    1

/** Maximum supported PWM frequency */
#define PWM_MAX_FREQUENCY_HZ    1000000

typedef struct {
    uint32_t timer;
    enum rcc_periph_clken clken;
    enum rcc_periph_rst rst;
    bool apb2;                  /**< Timer is clocked from APB2 (else APB1) */
    uint8_t af;                 /**< Output alternate function (STM32F4) */
} PWM_TimerInfo_t;

static const PWM_TimerInfo_t pwm_timers[] = {
#if defined(STM32F1)
    { TIM1,  RCC_TIM1,  RST_TIM1,  true,  0 },
    { TIM2,  RCC_TIM2,  RST_TIM2,  false, 0 },
    { TIM3,  RCC_TIM3,  RST_TIM3,  false, 0 },
    { TIM4,  RCC_TIM4,  RST_TIM4,  false, 0 },
#else
    { TIM1,  RCC_TIM1,  RST_TIM1,  true,  GPIO_AF1 },
    { TIM2,  RCC_TIM2,  RST_TIM2,  false, GPIO_AF1 },
    { TIM3,  RCC_TIM3,  RST_TIM3,  false, GPIO_AF2 },
    { TIM4,  RCC_TIM4,  RST_TIM4,  false, GPIO_AF2 },
    { TIM5,  RCC_TIM5,  RST_TIM5,  false, GPIO_AF2 },
    { TIM9,  RCC_TIM9,  RST_TIM9,  true,  GPIO_AF3 },
    { TIM10, RCC_TIM10, RST_TIM10, true,  GPIO_AF3 },
    { TIM11, RCC_TIM11, RST_TIM11, true,  GPIO_AF3 },
#endif // defined(STM32F1)
};

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Look up a supported timer, NULL if unsupported
 */
static const PWM_TimerInfo_t *pwm_find_timer(uint32_t timer) {
    for (size_t i = 0; i < sizeof(pwm_timers) / sizeof(pwm_timers[0]); i++) {
        if (pwm_timers[i].timer == timer) {
            return &pwm_timers[i];
        }
    }
    return NULL;
}

/**
 * @brief Timer input clock frequency
 */
static uint32_t pwm_timer_clock_hz(const PWM_TimerInfo_t *info) {
    uint32_t apb_hz = info->apb2 ? rcc_apb2_frequency : rcc_apb1_frequency;
    return (apb_hz == rcc_ahb_frequency) ? apb_hz : 2 * apb_hz;
}

/**
 * @brief Convert PWM_Channel_t to timer output compare channel
 */
static enum tim_oc_id pwm_get_oc_id(PWM_Channel_t ch) {
    switch (ch) {
        case PWM_CHANNEL_1: return TIM_OC1;
        case PWM_CHANNEL_2: return TIM_OC2;
        case PWM_CHANNEL_3: return TIM_OC3;
        case PWM_CHANNEL_4: return TIM_OC4;
        default:            return TIM_OC1;
    }
}

/**
 * @brief Get RCC for GPIO port
 */
static enum rcc_periph_clken pwm_get_gpio_rcc(uint32_t port) {
    switch (port) {
        case GPIOA: return RCC_GPIOA;
        case GPIOB: return RCC_GPIOB;
        case GPIOC: return RCC_GPIOC;
        case GPIOD: return RCC_GPIOD;
        case GPIOE: return RCC_GPIOE;
        default:    return RCC_GPIOA;
    }
}

/**
 * @brief Calculate prescaler for desired frequency
 *
 * @param clock_hz      Timer input clock
 * @param frequency_hz  Target frequency
 * @param resolution    Period value (resolution)
 * @return Prescaler value
 */
static uint16_t pwm_calculate_prescaler(uint32_t clock_hz, uint32_t frequency_hz,
                                        uint16_t resolution) {
    /*
     * PWM_freq = Timer_Clock / ((PSC + 1) * (ARR + 1))
     * PSC = (Timer_Clock / (PWM_freq * (ARR + 1))) - 1
     */
    uint32_t prescaler = (clock_hz / (frequency_hz * (resolution + 1))) - 1;

    /* Clamp to 16-bit range */
    if (prescaler > 0xFFFF) {
        prescaler = 0xFFFF;
    }

    return (uint16_t)prescaler;
}

/* ==========================================================================
 * Timer Initialization
 * ========================================================================== */

PWM_Status_t pwm_timer_init(PWM_Timer_t *pwm, uint32_t timer,
                            uint32_t frequency_hz, uint16_t resolution) {
    const PWM_TimerInfo_t *info = pwm_find_timer(timer);

    /* Validate parameters */
    if (!info) {
        return PWM_INVALID_TIMER;
    }

    if (frequency_hz < PWM_MIN_FREQUENCY_HZ || frequency_hz > PWM_MAX_FREQUENCY_HZ) {
        return PWM_INVALID_FREQUENCY;
    }

    /* Enable timer clock */
    rcc_periph_clock_enable(info->clken);
#if defined(STM32F1)
    rcc_periph_clock_enable(RCC_AFIO);
#endif // defined(STM32F1)

    /* Reset timer to default state */
    timer_disable_counter(timer);
    rcc_periph_reset_pulse(info->rst);

    /* Configure timer base */
    timer_set_mode(timer,
                   TIM_CR1_CKD_CK_INT,    /* No clock division */
                   TIM_CR1_CMS_EDGE,       /* Edge-aligned mode */
                   TIM_CR1_DIR_UP);        /* Count up */

    /* Calculate and set prescaler */
    uint16_t prescaler = pwm_calculate_prescaler(pwm_timer_clock_hz(info),
                                                 frequency_hz, resolution);
    timer_set_prescaler(timer, prescaler);

    /* Set period (auto-reload value) */
    timer_set_period(timer, resolution - 1);

    /* Enable preload for smooth updates */
    timer_enable_preload(timer);

    /* Continuous mode (auto-reload) */
    timer_continuous_mode(timer);

    /* Advanced timers gate all outputs behind the main output enable */
    if (timer == TIM1) {
        timer_enable_break_main_output(timer);
    }

    /* Generate update event to load prescaler */
    timer_generate_event(timer, TIM_EGR_UG);

    /* Store configuration */
    pwm->timer = timer;
    pwm->frequency_hz = frequency_hz;
    pwm->resolution = resolution;
    pwm->initialized = true;

    /* Start timer */
    timer_enable_counter(timer);

    return PWM_OK;
}

void pwm_timer_deinit(PWM_Timer_t *pwm) {
    if (!pwm || !pwm->initialized) {
        return;
    }

    timer_disable_counter(pwm->timer);
    rcc_periph_reset_pulse(pwm_find_timer(pwm->timer)->rst);

    pwm->initialized = false;
}

/* ==========================================================================
 * Channel Configuration
 * ========================================================================== */

PWM_Status_t pwm_channel_init(PWM_Channel_Config_t *channel, PWM_Timer_t *timer,
                              PWM_Channel_t ch, uint32_t gpio_port, uint16_t gpio_pin,
                              PWM_OutputMode_t mode) {
    if (!timer || !timer->initialized) {
        return PWM_NOT_INITIALIZED;
    }

    if (ch < PWM_CHANNEL_1 || ch > PWM_CHANNEL_4) {
        return PWM_INVALID_CHANNEL;
    }

    /* Enable GPIO clock */
    rcc_periph_clock_enable(pwm_get_gpio_rcc(gpio_port));

    /* Configure GPIO for alternate function output */
    gpio_compat_af_output(gpio_port, gpio_pin, pwm_find_timer(timer->timer)->af,
                          mode == PWM_OUTPUT_OPEN_DRAIN);

    /* Get output compare channel ID */
    enum tim_oc_id oc_id = pwm_get_oc_id(ch);

    /* Disable output compare channel during configuration */
    timer_disable_oc_output(timer->timer, oc_id);

    /* Configure output compare mode */
    timer_set_oc_mode(timer->timer, oc_id, TIM_OCM_PWM1);

    /* Enable output compare preload */
    timer_enable_oc_preload(timer->timer, oc_id);

    /* Set initial duty cycle to 0 */
    timer_set_oc_value(timer->timer, oc_id, 0);

    /* Store configuration */
    channel->timer = timer;
    channel->channel = ch;
    channel->gpio_port = gpio_port;
    channel->gpio_pin = gpio_pin;
    channel->mode = mode;
    channel->duty = 0;
    channel->enabled = false;

    return PWM_OK;
}

void pwm_channel_enable(PWM_Channel_Config_t *channel) {
    if (!channel || !channel->timer) {
        return;
    }

    enum tim_oc_id oc_id = pwm_get_oc_id(channel->channel);
    timer_enable_oc_output(channel->timer->timer, oc_id);
    channel->enabled = true;
}

void pwm_channel_disable(PWM_Channel_Config_t *channel) {
    if (!channel || !channel->timer) {
        return;
    }

    enum tim_oc_id oc_id = pwm_get_oc_id(channel->channel);
    timer_disable_oc_output(channel->timer->timer, oc_id);
    channel->enabled = false;
}

/* ==========================================================================
 * Duty Cycle Control
 * ========================================================================== */

void pwm_set_duty(PWM_Channel_Config_t *channel, uint16_t duty) {
    if (!channel || !channel->timer) {
        return;
    }

    /* Clamp to resolution */
    if (duty > channel->timer->resolution) {
        duty = channel->timer->resolution;
    }

    enum tim_oc_id oc_id = pwm_get_oc_id(channel->channel);
    timer_set_oc_value(channel->timer->timer, oc_id, duty);
    channel->duty = duty;
}

void pwm_set_duty_percent(PWM_Channel_Config_t *channel, uint16_t percent) {
    if (!channel || !channel->timer) {
        return;
    }

    /* percent is in tenths (0-1000 = 0.0% - 100.0%) */
    if (percent > 1000) {
        percent = 1000;
    }

    /* Calculate duty = (percent * resolution) / 1000 */
    uint32_t duty = ((uint32_t)percent * channel->timer->resolution) / 1000;

    pwm_set_duty(channel, (uint16_t)duty);
}

void pwm_set_duty_u8(PWM_Channel_Config_t *channel, uint8_t duty8) {
    if (!channel || !channel->timer) {
        return;
    }

    /* Map 0-255 to 0-resolution */
    uint32_t duty = ((uint32_t)duty8 * channel->timer->resolution) / 255;

    pwm_set_duty(channel, (uint16_t)duty);
}

uint16_t pwm_get_duty(const PWM_Channel_Config_t *channel) {
    if (!channel) {
        return 0;
    }
    return channel->duty;
}

/* ==========================================================================
 * Timer Control
 * ========================================================================== */

void pwm_timer_start(PWM_Timer_t *pwm) {
    if (!pwm || !pwm->initialized) {
        return;
    }
    timer_enable_counter(pwm->timer);
}

void pwm_timer_stop(PWM_Timer_t *pwm) {
    if (!pwm || !pwm->initialized) {
        return;
    }
    timer_disable_counter(pwm->timer);
}

PWM_Status_t pwm_set_frequency(PWM_Timer_t *pwm, uint32_t frequency_hz) {
    if (!pwm || !pwm->initialized) {
        return PWM_NOT_INITIALIZED;
    }

    if (frequency_hz < PWM_MIN_FREQUENCY_HZ || frequency_hz > PWM_MAX_FREQUENCY_HZ) {
        return PWM_INVALID_FREQUENCY;
    }

    /* Calculate new prescaler */
    uint16_t prescaler = pwm_calculate_prescaler(
        pwm_timer_clock_hz(pwm_find_timer(pwm->timer)), frequency_hz, pwm->resolution);

    /* Update prescaler (takes effect at next update event) */
    timer_set_prescaler(pwm->timer, prescaler);

    /* Update stored frequency */
    pwm->frequency_hz = frequency_hz;

    return PWM_OK;
}

uint32_t pwm_get_frequency(const PWM_Timer_t *pwm) {
    if (!pwm || !pwm->initialized) {
        return 0;
    }
    return pwm->frequency_hz;
}
