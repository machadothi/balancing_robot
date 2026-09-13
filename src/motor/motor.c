/**
 * @file motor.c
 * @brief TB6612FNG dual motor driver (Blue Pill board)
 *
 * Pin mapping:
 *
 *   TB6612FNG   STM32      Function
 *   ---------   -----      --------
 *   STBY        PB4        Standby release, high = driver enabled
 *   AIN1/AIN2   PB3/PA8    Left wheel (A) direction
 *   BIN1/BIN2   PB5/PB8    Right wheel (B) direction
 *   PWMA        PB1        Left wheel speed, TIM3_CH4
 *   PWMB        PB0        Right wheel speed, TIM3_CH3
 *   Encoder A   PA5        Rising edges, EXTI5
 *   Encoder B   PA6        Rising edges, EXTI6
 *
 * Direction truth table: IN1=H IN2=L forward, L/H reverse, H/H short brake,
 * L/L coast.
 *
 * @author Thiago Cunha
 * @date 2024
 */

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

#define MOTOR_PWM_FREQUENCY_HZ      1000    /* Suits JGA25-370 motors with the TB6612 */
#define MOTOR_PWM_RESOLUTION        1000

#define TB6612_STBY_PORT            GPIOB
#define TB6612_STBY_PIN             GPIO4

typedef struct {
    uint32_t in1_port;
    uint16_t in1_pin;
    uint32_t in2_port;
    uint16_t in2_pin;
    PWM_Channel_t pwm_channel;  /**< TIM3 channel, pin on port B */
    uint16_t pwm_pin;
    uint32_t exti;              /**< Encoder line, pin on port A */
    uint16_t encoder_pin;
} TB6612_Channel_t;

static const TB6612_Channel_t channels[MOTOR_COUNT] = {
    [MOTOR_LEFT]  = { GPIOB, GPIO3, GPIOA, GPIO8, PWM_CHANNEL_4, GPIO1, EXTI5, GPIO5 },
    [MOTOR_RIGHT] = { GPIOB, GPIO5, GPIOB, GPIO8, PWM_CHANNEL_3, GPIO0, EXTI6, GPIO6 },
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

static PWM_Timer_t pwm_timer;
static PWM_Channel_Config_t pwm[MOTOR_COUNT];
static volatile int32_t encoder_count[MOTOR_COUNT];

/* ==========================================================================
 * Interrupt Handler
 * ========================================================================== */

void exti9_5_isr(void) {
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (exti_get_flag_status(channels[m].exti)) {
            exti_reset_request(channels[m].exti);
            encoder_count[m]++;
        }
    }
}

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

static void set_inputs(const TB6612_Channel_t *ch, bool in1, bool in2) {
    if (in1) {
        gpio_set(ch->in1_port, ch->in1_pin);
    } else {
        gpio_clear(ch->in1_port, ch->in1_pin);
    }
    if (in2) {
        gpio_set(ch->in2_port, ch->in2_pin);
    } else {
        gpio_clear(ch->in2_port, ch->in2_pin);
    }
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void motor_init(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Direction and standby pins, driver held in standby during setup */
    gpio_set_mode(TB6612_STBY_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TB6612_STBY_PIN);
    gpio_clear(TB6612_STBY_PORT, TB6612_STBY_PIN);

    for (int m = 0; m < MOTOR_COUNT; m++) {
        const TB6612_Channel_t *ch = &channels[m];

        gpio_set_mode(ch->in1_port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ch->in1_pin);
        gpio_set_mode(ch->in2_port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ch->in2_pin);
        set_inputs(ch, false, false);

        /* Encoder: rising edges on EXTI */
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, ch->encoder_pin);
        exti_select_source(ch->exti, GPIOA);
        exti_set_trigger(ch->exti, EXTI_TRIGGER_RISING);
        exti_enable_request(ch->exti);
    }
    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x80);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);

    pwm_timer_init(&pwm_timer, TIM3, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION);
    for (int m = 0; m < MOTOR_COUNT; m++) {
        pwm_channel_init(&pwm[m], &pwm_timer, channels[m].pwm_channel,
                         GPIOB, channels[m].pwm_pin, PWM_OUTPUT_OPEN_DRAIN);
        pwm_channel_enable(&pwm[m]);
    }

    gpio_set(TB6612_STBY_PORT, TB6612_STBY_PIN);
}

void motor_set(Motor_Id_t id, int16_t command) {
    if (id >= MOTOR_COUNT) {
        return;
    }
    if (command > MOTOR_COMMAND_MAX) {
        command = MOTOR_COMMAND_MAX;
    } else if (command < -MOTOR_COMMAND_MAX) {
        command = -MOTOR_COMMAND_MAX;
    }

    set_inputs(&channels[id], command >= 0, command < 0);
    pwm_set_duty_u8(&pwm[id], (uint8_t)(command < 0 ? -command : command));
}

void motor_brake(Motor_Id_t id) {
    if (id < MOTOR_COUNT) {
        set_inputs(&channels[id], true, true);
    }
}

void motor_coast(Motor_Id_t id) {
    if (id < MOTOR_COUNT) {
        set_inputs(&channels[id], false, false);
    }
}

void motor_standby(bool enable) {
    if (enable) {
        gpio_clear(TB6612_STBY_PORT, TB6612_STBY_PIN);
    } else {
        gpio_set(TB6612_STBY_PORT, TB6612_STBY_PIN);
    }
}

void motor_emergency_stop(void) {
    gpio_clear(TB6612_STBY_PORT, TB6612_STBY_PIN);
    for (int m = 0; m < MOTOR_COUNT; m++) {
        pwm_set_duty(&pwm[m], 0);
    }
}

int32_t motor_get_encoder(Motor_Id_t id) {
    return (id < MOTOR_COUNT) ? encoder_count[id] : 0;
}

void motor_reset_encoder(Motor_Id_t id) {
    if (id < MOTOR_COUNT) {
        encoder_count[id] = 0;
    }
}
