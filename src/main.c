#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
// #include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "communication/uart.h"
#include "imu/mpu6050.h"
#include "log/log.h"
#include "motor/motor.h"
#include "robot/robot.h"

#define NO_OPT __attribute__((optimize("O0")))


extern void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName );

void
vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName ) {
    (void)xTask;
    (void)pcTaskName;
    for(;;);
}

/*********************************************************************
 * Blink LED:
 *********************************************************************/
static void setup_led(void ) {
    // LED GPIO BluePill
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

    // GPIOB Pins LED RGB
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO12); // green
    gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13); // Blue
    gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO14); // Red
}

static void
led(void *args) {
    (void)args;

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        gpio_toggle(GPIOC,GPIO13);

        gpio_set(GPIOB,GPIO12);
        gpio_set(GPIOB,GPIO13);
        gpio_toggle(GPIOB,GPIO14);
        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(250));
    }
}

// -----------------------------------------------------------------------------

int NO_OPT
main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz();    // Use this for "blue pill"

    // LED GPIO
    setup_led();

    uart_peripheral_setup();

    static LogDriver_t logDriver;
    logDriver.log_level = OFF;
    logDriver.send = uart_puts;
    
    log_init(&logDriver);

    xTaskCreate(led,"LED",50,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(uart_task,"UART",150,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(imu_task,"IMU",800,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(robot_task,"IMU",800,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(motor_demo_task,"MOTOR",300,NULL,configMAX_PRIORITIES-1,NULL);
    
    vTaskStartScheduler();
    for (;;);

    return 0;
}

// End
