#include <stdlib.h>

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

    IMU *imu = get_mpu6050_imu();

    // IMU - needs to be initialized after the scheduler.
    // initialize();
    imu_init(imu);

    char buffer[7];  // Large enough for a 2-byte int and '\0'

    for (;;) {
        TickType_t LastWakeTime = xTaskGetTickCount();

        uint8_t data = imu_id(imu);
        itoa(data, buffer, 16); // 10 for base 10 (decimal) representation

        uart_puts("  device id: 0x");
        uart_puts(buffer);
        uart_puts(".\n\n\r");

        int16_t acc_x = imu_acc_x(imu);
        itoa(acc_x, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_x: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t acc_y = imu_acc_y(imu);
        itoa(acc_y, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_y: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t acc_z = imu_acc_z(imu);
        itoa(acc_z, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  acc_z: ");
        uart_puts(buffer);
        uart_puts(".\n\n\r");


        int16_t gyro_x = imu_gyro_x(imu);
        itoa(gyro_x, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_x: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t gyro_y = imu_gyro_y(imu);
        itoa(gyro_y, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_y: ");
        uart_puts(buffer);
        uart_puts(".\n\r");

        int16_t gyro_z = imu_gyro_z(imu);
        itoa(gyro_z, buffer, 10); // 10 for base 10 (decimal) representation

        uart_puts("  gyro_z: ");
        uart_puts(buffer);
        uart_puts(".\n\n\r");

        vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(200));
    }
}

// -----------------------------------------------------------------------------

int
main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz();    // Use this for "blue pill"

    // LED GPIO
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

    // UART
    uart_setup();

    xTaskCreate(led,"LED",30,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(uart_task,"UART",50,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(demo_task,"DEMO",150,NULL,configMAX_PRIORITIES-1,NULL);
    
    vTaskStartScheduler();
    for (;;);

    return 0;
}

// End
