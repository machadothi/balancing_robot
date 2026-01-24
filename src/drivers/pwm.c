/**
 * @file pwm.c
 * @brief PWM (Pulse Width Modulation) Driver Implementation
 * 
 * Timer-based PWM generation for STM32F103. Uses general-purpose timers
 * (TIM2, TIM3, TIM4) in PWM mode 1 with configurable frequency and resolution.
 * 
 * Clock Configuration (assuming 72MHz system clock):
 *   - APB1 timers (TIM2/3/4): 72MHz (APB1 prescaler = 2, timer clock x2)
 *   - PWM frequency = Timer_Clock / ((Prescaler + 1) * (Period + 1))
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "drivers/pwm.h"

/* ==========================================================================
 * Private Definitions
 * ========================================================================== */

/** Timer input clock frequency (APB1 timers with APB1 prescaler > 1) */
#define PWM_TIMER_CLOCK_HZ      72000000UL

/** Minimum supported PWM frequency */
#define PWM_MIN_FREQUENCY_HZ    1

/** Maximum supported PWM frequency */
#define PWM_MAX_FREQUENCY_HZ    1000000

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Get the RCC clock enable for a timer
 */
static enum rcc_periph_clken pwm_get_timer_rcc(uint32_t timer) {
    switch (timer) {
        case TIM2: return RCC_TIM2;
        case TIM3: return RCC_TIM3;
        case TIM4: return RCC_TIM4;
        default:   return RCC_TIM2;  /* Fallback */
    }
}

/**
 * @brief Get the RCC reset for a timer
 */
static enum rcc_periph_rst pwm_get_timer_rst(uint32_t timer) {
    switch (timer) {
        case TIM2: return RST_TIM2;
        case TIM3: return RST_TIM3;
        case TIM4: return RST_TIM4;
        default:   return RST_TIM2;
    }
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
        default:    return RCC_GPIOA;
    }
}

/**
 * @brief Calculate prescaler for desired frequency
 * 
 * @param frequency_hz  Target frequency
 * @param resolution    Period value (resolution)
 * @return Prescaler value
 */
static uint16_t pwm_calculate_prescaler(uint32_t frequency_hz, uint16_t resolution) {
    /*
     * PWM_freq = Timer_Clock / ((PSC + 1) * (ARR + 1))
     * PSC = (Timer_Clock / (PWM_freq * (ARR + 1))) - 1
     */
    uint32_t prescaler = (PWM_TIMER_CLOCK_HZ / (frequency_hz * (resolution + 1))) - 1;
    
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
    /* Validate parameters */
    if (timer != TIM2 && timer != TIM3 && timer != TIM4) {
        return PWM_INVALID_TIMER;
    }
    
    if (frequency_hz < PWM_MIN_FREQUENCY_HZ || frequency_hz > PWM_MAX_FREQUENCY_HZ) {
        return PWM_INVALID_FREQUENCY;
    }
    
    /* Enable timer clock */
    rcc_periph_clock_enable(pwm_get_timer_rcc(timer));
    rcc_periph_clock_enable(RCC_AFIO);
    
    /* Reset timer to default state */
    timer_disable_counter(timer);
    rcc_periph_reset_pulse(pwm_get_timer_rst(timer));
    
    /* Configure timer base */
    timer_set_mode(timer, 
                   TIM_CR1_CKD_CK_INT,    /* No clock division */
                   TIM_CR1_CMS_EDGE,       /* Edge-aligned mode */
                   TIM_CR1_DIR_UP);        /* Count up */
    
    /* Calculate and set prescaler */
    uint16_t prescaler = pwm_calculate_prescaler(frequency_hz, resolution);
    timer_set_prescaler(timer, prescaler);
    
    /* Set period (auto-reload value) */
    timer_set_period(timer, resolution - 1);
    
    /* Enable preload for smooth updates */
    timer_enable_preload(timer);
    
    /* Continuous mode (auto-reload) */
    timer_continuous_mode(timer);
    
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
    rcc_periph_reset_pulse(pwm_get_timer_rst(pwm->timer));
    
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
    uint8_t gpio_cnf = (mode == PWM_OUTPUT_PUSH_PULL) ? 
                        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL :
                        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN;
    
    gpio_set_mode(gpio_port, GPIO_MODE_OUTPUT_50_MHZ, gpio_cnf, gpio_pin);
    
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
    uint16_t prescaler = pwm_calculate_prescaler(frequency_hz, pwm->resolution);
    
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
