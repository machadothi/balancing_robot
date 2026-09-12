/**
 * @file motor_hiwonder.c
 * @brief Motor driver for the Hiwonder ROS Robot Control Board (STM32F407)
 *
 * Implements the motor.h API for the board's encoder motor ports. Each port's
 * H-bridge has two PWM inputs: PWM on the forward input with the reverse
 * input low drives forward, and the other way round for reverse. Both low
 * coasts, both high brakes. Encoders are quadrature, counted by a timer in
 * encoder mode (x4).
 *
 * Pin mapping from the vendor firmware (RosRobotControllerM4):
 *
 *   Port  Forward PWM        Reverse PWM        Encoder A / B
 *   ----  -----------        -----------        -------------
 *   M1    TIM1_CH4  PE14     TIM1_CH3  PE13     TIM5  PA0  / PA1
 *   M2    TIM1_CH2  PE11     TIM1_CH1  PE9      TIM2  PA15 / PB3
 *   M3    TIM9_CH1  PE5      TIM9_CH2  PE6      TIM4  PB6  / PB7
 *   M4    TIM11_CH1 PB9      TIM10_CH1 PB8      TIM3  PB4  / PB5
 *
 * The ports used for motor1 and motor2 are set in board_config.h.
 */

#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "board_config.h"
#include "motor/motor.h"
#include "drivers/pwm.h"

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define MOTOR_PWM_FREQUENCY_HZ      1000
#define MOTOR_PWM_RESOLUTION        1000

#define MOTOR_COUNT                 2

/* ==========================================================================
 * Port Table
 * ========================================================================== */

typedef struct {
    uint32_t timer;
    PWM_Channel_t channel;
    uint32_t port;
    uint16_t pin;
} MotorPwmPin_t;

typedef struct {
    MotorPwmPin_t forward;
    MotorPwmPin_t reverse;
    uint32_t enc_timer;
    enum rcc_periph_clken enc_rcc;
    uint8_t enc_af;
    uint32_t enc_port_a;
    uint16_t enc_pin_a;
    uint32_t enc_port_b;
    uint16_t enc_pin_b;
} MotorPort_t;

static const MotorPort_t motor_ports[] = {
    { {TIM1,  PWM_CHANNEL_4, GPIOE, GPIO14}, {TIM1,  PWM_CHANNEL_3, GPIOE, GPIO13},
      TIM5, RCC_TIM5, GPIO_AF2, GPIOA, GPIO0,  GPIOA, GPIO1 },
    { {TIM1,  PWM_CHANNEL_2, GPIOE, GPIO11}, {TIM1,  PWM_CHANNEL_1, GPIOE, GPIO9},
      TIM2, RCC_TIM2, GPIO_AF1, GPIOA, GPIO15, GPIOB, GPIO3 },
    { {TIM9,  PWM_CHANNEL_1, GPIOE, GPIO5},  {TIM9,  PWM_CHANNEL_2, GPIOE, GPIO6},
      TIM4, RCC_TIM4, GPIO_AF2, GPIOB, GPIO6,  GPIOB, GPIO7 },
    { {TIM11, PWM_CHANNEL_1, GPIOB, GPIO9},  {TIM10, PWM_CHANNEL_1, GPIOB, GPIO8},
      TIM3, RCC_TIM3, GPIO_AF2, GPIOB, GPIO4,  GPIOB, GPIO5 },
};

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

typedef struct {
    const MotorPort_t *port;
    PWM_Channel_Config_t forward;
    PWM_Channel_Config_t reverse;
    bool direction;
    uint8_t speed;
} Motor_t;

static Motor_t motors[MOTOR_COUNT];

/** One entry per timer used by the selected ports (at most 2 per motor) */
static PWM_Timer_t pwm_timers[2 * MOTOR_COUNT];

static bool motors_standby = false;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

static PWM_Timer_t *motor_pwm_timer(uint32_t timer) {
    for (size_t i = 0; i < 2 * MOTOR_COUNT; i++) {
        if (pwm_timers[i].initialized && pwm_timers[i].timer == timer) {
            return &pwm_timers[i];
        }
    }

    for (size_t i = 0; i < 2 * MOTOR_COUNT; i++) {
        if (!pwm_timers[i].initialized) {
            pwm_timer_init(&pwm_timers[i], timer, MOTOR_PWM_FREQUENCY_HZ,
                           MOTOR_PWM_RESOLUTION);
            return &pwm_timers[i];
        }
    }

    return NULL;
}

static void motor_pwm_pin_init(PWM_Channel_Config_t *channel, const MotorPwmPin_t *pin) {
    pwm_channel_init(channel, motor_pwm_timer(pin->timer), pin->channel,
                     pin->port, pin->pin, PWM_OUTPUT_PUSH_PULL);
    pwm_channel_enable(channel);
}

static void motor_encoder_init(const MotorPort_t *port) {
    rcc_periph_clock_enable(port->enc_rcc);

    gpio_mode_setup(port->enc_port_a, GPIO_MODE_AF, GPIO_PUPD_PULLUP, port->enc_pin_a);
    gpio_set_af(port->enc_port_a, port->enc_af, port->enc_pin_a);
    gpio_mode_setup(port->enc_port_b, GPIO_MODE_AF, GPIO_PUPD_PULLUP, port->enc_pin_b);
    gpio_set_af(port->enc_port_b, port->enc_af, port->enc_pin_b);

    /* TIM2 and TIM5 are 32-bit, the others 16-bit */
    bool wide = (port->enc_timer == TIM2 || port->enc_timer == TIM5);
    timer_set_period(port->enc_timer, wide ? 0xFFFFFFFF : 0xFFFF);

    timer_slave_set_mode(port->enc_timer, TIM_SMCR_SMS_EM3);
    timer_ic_set_input(port->enc_timer, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(port->enc_timer, TIM_IC2, TIM_IC_IN_TI2);
    timer_set_counter(port->enc_timer, 0);
    timer_enable_counter(port->enc_timer);
}

static void motor_apply(Motor_t *motor) {
    uint8_t speed = motors_standby ? 0 : motor->speed;
    PWM_Channel_Config_t *on = motor->direction ? &motor->forward : &motor->reverse;
    PWM_Channel_Config_t *off = motor->direction ? &motor->reverse : &motor->forward;

    /* Clear the opposite input first so both are never driven together */
    pwm_set_duty(off, 0);
    pwm_set_duty_u8(on, speed);
}

static void motor_set_inputs(Motor_t *motor, uint16_t duty) {
    pwm_set_duty(&motor->forward, duty);
    pwm_set_duty(&motor->reverse, duty);
}

/* ==========================================================================
 * Public Functions - Initialization
 * ========================================================================== */

void motor_init(void) {
    const uint8_t port_numbers[MOTOR_COUNT] = { BOARD_MOTOR1_PORT, BOARD_MOTOR2_PORT };

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        Motor_t *motor = &motors[i];

        motor->port = &motor_ports[port_numbers[i] - 1];
        motor->direction = true;
        motor->speed = 0;

        motor_pwm_pin_init(&motor->forward, &motor->port->forward);
        motor_pwm_pin_init(&motor->reverse, &motor->port->reverse);
        motor_encoder_init(motor->port);
    }

    motors_standby = false;
}

void motor_deinit(void) {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwm_channel_disable(&motors[i].forward);
        pwm_channel_disable(&motors[i].reverse);
        timer_disable_counter(motors[i].port->enc_timer);
    }

    for (size_t i = 0; i < 2 * MOTOR_COUNT; i++) {
        pwm_timer_deinit(&pwm_timers[i]);
    }
}

/* ==========================================================================
 * Public Functions - Motor Control
 * ========================================================================== */

void motor1_set_direction(bool direction) {
    motors[0].direction = direction;
    motor_apply(&motors[0]);
}

void motor2_set_direction(bool direction) {
    motors[1].direction = direction;
    motor_apply(&motors[1]);
}

void motor1_set_speed(uint8_t speed) {
    motors[0].speed = speed;
    motor_apply(&motors[0]);
}

void motor2_set_speed(uint8_t speed) {
    motors[1].speed = speed;
    motor_apply(&motors[1]);
}

void motor1_brake(void) {
    motor_set_inputs(&motors[0], MOTOR_PWM_RESOLUTION);
}

void motor2_brake(void) {
    motor_set_inputs(&motors[1], MOTOR_PWM_RESOLUTION);
}

void motor1_coast(void) {
    motor_set_inputs(&motors[0], 0);
}

void motor2_coast(void) {
    motor_set_inputs(&motors[1], 0);
}

void motor_standby(bool enable) {
    motors_standby = enable;
    motor_apply(&motors[0]);
    motor_apply(&motors[1]);
}

/* ==========================================================================
 * Public Functions - Encoder
 * ========================================================================== */

uint32_t motor1_get_encoder(void) {
    return timer_get_counter(motors[0].port->enc_timer);
}

uint32_t motor2_get_encoder(void) {
    return timer_get_counter(motors[1].port->enc_timer);
}

void motor1_reset_encoder(void) {
    timer_set_counter(motors[0].port->enc_timer, 0);
}

void motor2_reset_encoder(void) {
    timer_set_counter(motors[1].port->enc_timer, 0);
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
    motor1_set_speed(128);
    motor2_set_speed(128);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
