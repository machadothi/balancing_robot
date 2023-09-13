#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "imu/mpu6050.h"

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
static void
led(void *args) {
    (void)args;

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        gpio_toggle(GPIOC,GPIO13);
        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(250));
    }
}

static QueueHandle_t uart_txq;        // TX queue for UART

/*********************************************************************
 * Configure and initialize USART2:
 *********************************************************************/
static void
uart_setup(void) {

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    // UART TX on PA9 (GPIO_USART2_TX)
    gpio_set_mode(GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART2_TX);

    usart_set_baudrate(USART2,38400);
    usart_set_databits(USART2,8);
    usart_set_stopbits(USART2,USART_STOPBITS_1);
    usart_set_mode(USART2,USART_MODE_TX);
    usart_set_parity(USART2,USART_PARITY_NONE);
    usart_set_flow_control(USART2,USART_FLOWCONTROL_NONE);
    usart_enable(USART2);

    // Create a queue for data to transmit from UART
    uart_txq = xQueueCreate(256,sizeof(char));
}

/*********************************************************************
 * USART Task: 
 *********************************************************************/
static void
uart_task(void *args __attribute__((unused))) {
    char ch;

    for (;;) {
        // Receive char to be TX
        if ( xQueueReceive(uart_txq,&ch,500) == pdPASS ) {
            while ( !usart_get_flag(USART2,USART_SR_TXE) )
                taskYIELD();    // Yield until ready
            usart_send(USART2,ch);
        }
    }
}

/*********************************************************************
 * Queue a string of characters to be TX
 *********************************************************************/
static void
uart_puts(const char *s) {
    
    for ( ; *s; ++s ) {
        // blocks when queue is full
        xQueueSend(uart_txq,s,portMAX_DELAY); 
    }
}

/*********************************************************************
 * Demo Task:
 *    Simply queues up two line messages to be TX, one second
 *    apart.
 *********************************************************************/
static void
demo_task(void *args __attribute__((unused))) {

    // IMU - needs to be initialized after the scheduler.
    initialize();

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        uart_puts("Now this is a message..\n\r");
        uart_puts("  sent via FreeRTOS queues.\n\n\r");

        // uint8_t data = getDeviceID();
        // uart_puts("  device id: ");
        // uart_puts(&data);
        // uart_puts(".\n\n\r");
        
        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(1000));
    }
}

int
main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz();    // Use this for "blue pill"

    // LED GPIO
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

    // I2c
    rcc_periph_clock_enable(RCC_GPIOB);	// I2C
    rcc_periph_clock_enable(RCC_I2C1);	// I2C
    gpio_set_mode(GPIOB,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
        GPIO6|GPIO7);            // I2C
    gpio_set(GPIOB,GPIO6|GPIO7);        // Idle high

    // UART
    uart_setup();

    xTaskCreate(led,"LED",100,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(uart_task,"UART",100,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(demo_task,"DEMO",100,NULL,configMAX_PRIORITIES-1,NULL);
    
    vTaskStartScheduler();
    for (;;);

    return 0;
}

// End
