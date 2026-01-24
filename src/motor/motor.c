/**
 * @file motor.c
 * @brief TB6612FNG Motor Driver Implementation
 * 
 * Dual DC motor control using TB6612FNG H-bridge driver IC.
 * Uses the PWM driver for speed control via TIM3 channels.
 * 
 * Hardware Configuration:
 *   - Motor A: AIN1/AIN2 for direction, PWMA for speed
 *   - Motor B: BIN1/BIN2 for direction, PWMB for speed
 *   - STBY: Active-high standby release
 *   - Encoders: EXTI interrupts for speed feedback
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>

#include "motor/motor.h"
#include "drivers/pwm.h"

/* ==========================================================================
 * Configuration
 * ========================================================================== */

/** PWM frequency for motor control (Hz) */
#define MOTOR_PWM_FREQUENCY_HZ      20000

/** PWM resolution (1000 = 0.1% steps) */
#define MOTOR_PWM_RESOLUTION        1000

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

/** PWM timer instance */
static PWM_Timer_t motor_pwm_timer;

/** PWM channel for Motor A (TIM3_CH4 on PB1) */
static PWM_Channel_Config_t motor_a_pwm;

/** PWM channel for Motor B (TIM3_CH3 on PB0) */
static PWM_Channel_Config_t motor_b_pwm;

/** Encoder counters */
static volatile uint32_t motor_a_encoder_count = 0;
static volatile uint32_t motor_b_encoder_count = 0;

/* ==========================================================================
 * Private Function Prototypes
 * ========================================================================== */

static void motor_gpio_init(void);
static void motor_encoder_init(void);
static void motor_pwm_init(void);

/* ==========================================================================
 * Interrupt Handlers
 * ========================================================================== */

/**
 * @brief Encoder interrupt handler (EXTI5-9)
 * 
 * Handles rising edge interrupts from motor encoders for
 * speed measurement and position tracking.
 */
void exti9_5_isr(void) {
    if (exti_get_flag_status(EXTI5)) {
        exti_reset_request(EXTI5);
        motor_a_encoder_count++;
    }
    
    if (exti_get_flag_status(EXTI6)) {
        exti_reset_request(EXTI6);
        motor_b_encoder_count++;
    }
}

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

/**
 * @brief Initialize motor control GPIO pins
 * 
 * Configures direction control pins (AIN1, AIN2, BIN1, BIN2) and
 * standby control pin as push-pull outputs.
 */
static void motor_gpio_init(void) {
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    
    /* Configure direction control pins on Port B */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  TB6612_STBY | TB6612_AIN1 | TB6612_BIN1 | TB6612_BIN2);
    
    /* Configure AIN2 on Port A */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  TB6612_AIN2);
    
    /* Start in standby mode (motors disabled) */
    gpio_clear(GPIOB, TB6612_STBY);
    
    /* Set initial direction (both motors stopped, direction neutral) */
    gpio_clear(GPIOB, TB6612_AIN1 | TB6612_BIN1 | TB6612_BIN2);
    gpio_clear(GPIOA, TB6612_AIN2);
}

/**
 * @brief Initialize encoder inputs with EXTI interrupts
 */
static void motor_encoder_init(void) {
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
    
    /* Configure encoder pins as floating inputs */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                  MOTOR1_ENCODER | MOTOR2_ENCODER);
    
    /* Configure EXTI for encoder pins */
    exti_select_source(EXTI5, GPIOA);
    exti_select_source(EXTI6, GPIOA);
    
    exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
    exti_set_trigger(EXTI6, EXTI_TRIGGER_RISING);
    
    exti_enable_request(EXTI5);
    exti_enable_request(EXTI6);
    
    /* Enable EXTI interrupt with medium priority */
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x80);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

/**
 * @brief Initialize PWM for motor speed control
 */
static void motor_pwm_init(void) {
    /* Initialize PWM timer (TIM3) */
    pwm_timer_init(&motor_pwm_timer, TIM3, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION);
    
    /* Initialize Motor A PWM channel (TIM3_CH4 on PB1) */
    pwm_channel_init(&motor_a_pwm, &motor_pwm_timer, PWM_CHANNEL_4,
                     GPIOB, TB6612_PWMA, PWM_OUTPUT_OPEN_DRAIN);
    pwm_channel_enable(&motor_a_pwm);
    
    /* Initialize Motor B PWM channel (TIM3_CH3 on PB0) */
    pwm_channel_init(&motor_b_pwm, &motor_pwm_timer, PWM_CHANNEL_3,
                     GPIOB, TB6612_PWMB, PWM_OUTPUT_OPEN_DRAIN);
    pwm_channel_enable(&motor_b_pwm);
}

/* ==========================================================================
 * Public Functions - Initialization
 * ========================================================================== */

void motor_init(void) {
    motor_gpio_init();
    motor_encoder_init();
    motor_pwm_init();
    
    /* Exit standby mode - enable motor driver */
    gpio_set(GPIOB, TB6612_STBY);
}

void motor_deinit(void) {
    /* Enter standby mode */
    gpio_clear(GPIOB, TB6612_STBY);
    
    /* Stop PWM */
    pwm_channel_disable(&motor_a_pwm);
    pwm_channel_disable(&motor_b_pwm);
    pwm_timer_deinit(&motor_pwm_timer);
}

/* ==========================================================================
 * Public Functions - Motor Control
 * ========================================================================== */

void motor1_set_direction(bool direction) {
    if (direction) {
        gpio_set(GPIOB, TB6612_AIN1);
        gpio_clear(GPIOA, TB6612_AIN2);
    } else {
        gpio_clear(GPIOB, TB6612_AIN1);
        gpio_set(GPIOA, TB6612_AIN2);
    }
}

void motor2_set_direction(bool direction) {
    if (direction) {
        gpio_set(GPIOB, TB6612_BIN1);
        gpio_clear(GPIOB, TB6612_BIN2);
    } else {
        gpio_clear(GPIOB, TB6612_BIN1);
        gpio_set(GPIOB, TB6612_BIN2);
    }
}

void motor1_set_speed(uint8_t speed) {
    pwm_set_duty_u8(&motor_a_pwm, speed);
}

void motor2_set_speed(uint8_t speed) {
    pwm_set_duty_u8(&motor_b_pwm, speed);
}

void motor1_brake(void) {
    /* Short brake: both direction pins high */
    gpio_set(GPIOB, TB6612_AIN1);
    gpio_set(GPIOA, TB6612_AIN2);
}

void motor2_brake(void) {
    /* Short brake: both direction pins high */
    gpio_set(GPIOB, TB6612_BIN1);
    gpio_set(GPIOB, TB6612_BIN2);
}

void motor1_coast(void) {
    /* Coast: both direction pins low */
    gpio_clear(GPIOB, TB6612_AIN1);
    gpio_clear(GPIOA, TB6612_AIN2);
}

void motor2_coast(void) {
    /* Coast: both direction pins low */
    gpio_clear(GPIOB, TB6612_BIN1);
    gpio_clear(GPIOB, TB6612_BIN2);
}

void motor_standby(bool enable) {
    if (enable) {
        gpio_clear(GPIOB, TB6612_STBY);
    } else {
        gpio_set(GPIOB, TB6612_STBY);
    }
}

/* ==========================================================================
 * Public Functions - Encoder
 * ========================================================================== */

uint32_t motor1_get_encoder(void) {
    return motor_a_encoder_count;
}

uint32_t motor2_get_encoder(void) {
    return motor_b_encoder_count;
}

void motor1_reset_encoder(void) {
    motor_a_encoder_count = 0;
}

void motor2_reset_encoder(void) {
    motor_b_encoder_count = 0;
}

/* ==========================================================================
 * Demo Task
 * ========================================================================== */

void motor_demo_task(void *args) {
    (void)args;
    
    motor_init();
    
    /* Set both motors forward at 50% speed */
    motor1_set_direction(true);
    motor2_set_direction(true);
    motor1_set_speed(128);  /* 50% */
    motor2_set_speed(128);
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
