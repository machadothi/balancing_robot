#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "communication/uart.h"
#include "log/log.h"

// -----------------------------------------------------------------------------

QueueHandle_t uart_txq;

// -----------------------------------------------------------------------------

void
uart_peripheral_setup(void) {
    log_message(INFO,UART_BUS,"Initializing UART");
    
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

// -----------------------------------------------------------------------------

void
uart_puts(const char *s) {

    for ( ; *s; ++s ) {
        // blocks when queue is full
        xQueueSend(uart_txq,s,portMAX_DELAY); 
    }
}

// -----------------------------------------------------------------------------

void
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