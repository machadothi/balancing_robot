#include <FreeRTOS.h>
#include <task.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include "motor.h"

static void pwm_init(void);
static void pwm_set_duty_cycle(uint32_t pin, uint8_t duty_cycle);

void motor_init(void) {
    // outputs
    // gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 
    //   TB6612_STBY | TB6612_AIN1 | TB6612_BIN1 | TB6612_BIN2);

    // gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 
    //   TB6612_AIN2);
    
    // inputs
    // gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, MOTOR1 | MOTOR2);
    // exti_select_source(EXTI5 | EXTI6, GPIOA);
    // exti_set_trigger(EXTI5 | EXTI6, EXTI_TRIGGER_RISING);
    // exti_enable_request(EXTI5 | EXTI6);
    // nvic_enable_irq(NVIC_EXTI5_IRQ);
    // nvic_enable_irq(NVIC_EXTI6_IRQ);

    pwm_init();
}

void motor1_set_direction(bool direction) {
    if (direction) {
        gpio_set(GPIOB, TB6612_AIN1);
        gpio_clear(GPIOB, TB6612_AIN2);
    } else {
        gpio_clear(GPIOB, TB6612_AIN1);
        gpio_set(GPIOB, TB6612_AIN2);
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
    pwm_set_duty_cycle(TB6612_PWMA, speed);
}

void motor2_set_speed(uint8_t speed) {
    pwm_set_duty_cycle(TB6612_PWMB, speed);
}

static void 
pwm_init(void) {
    rcc_periph_clock_enable(RCC_TIM3);		// Need TIM3 clock
    rcc_periph_clock_enable(RCC_AFIO);		// Need AFIO clock
    rcc_periph_clock_enable(RCC_GPIOB);		// Need GPIOB clock

	gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ,	// High speed
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,GPIO0);	// PB0=TIM3.CH3

	gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ,	// High speed
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,GPIO1);	// PB1=TIM3.CH4

    timer_disable_counter(TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_mode(TIM3, 
        TIM_CR1_CKD_CK_INT, 
        TIM_CR1_CMS_EDGE, 
        TIM_CR1_DIR_UP);
    
    timer_set_prescaler(TIM3, 36000000*2/150/50-1); // 500Hz
    timer_enable_preload(TIM3);
    timer_continuous_mode(TIM3);
    timer_set_period(TIM3, 50-1);

    timer_disable_oc_output(TIM3, TIM_OC3);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC3);

    timer_disable_oc_output(TIM3, TIM_OC4);
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC4);

    timer_set_oc_value(TIM3, TIM_OC3, 0);
    timer_set_oc_value(TIM3, TIM_OC4, 0);
    timer_enable_counter(TIM3);
}

static void
pwm_set_duty_cycle(uint32_t pin, uint8_t duty_cycle) {
    switch (pin)
    {
        case TB6612_PWMA:
            timer_set_oc_value(TIM3, TIM_OC4, duty_cycle);
            break;

        case TB6612_PWMB:
            timer_set_oc_value(TIM3, TIM_OC3, duty_cycle);
            break;
        
        default:
            break;
    }
}

void
motor_demo_task(void *args __attribute__((unused))) {
    motor_init();

    static const int ms[4] = { 20000, 15000, 5000, 1000 };
    static int i = 0;
    for (;;) {
        pwm_set_duty_cycle(TB6612_PWMA, 10); // 20%
        pwm_set_duty_cycle(TB6612_PWMB, 30); // 60%
        i++;
        if (i > 3)
            i = 0;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

