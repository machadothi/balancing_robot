/**
 * @file interrupts.c
 * @brief Interrupt Service Routines (ISRs)
 *
 * Contains ISR implementations for DMA, I2C, and UART peripherals.
 * These override the weak default handlers from libopencm3; the handler
 * names come from board_config.h.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "board_config.h"
#include "drivers/i2c.h"
#include "drivers/uart.h"

/* ==========================================================================
 * I2C Event and Error Interrupt Handlers
 * ========================================================================== */

void BOARD_I2C_EV_ISR(void) {
    i2c_ev_isr(&i2c_board_bus);
}

void BOARD_I2C_ER_ISR(void) {
    i2c_er_isr(&i2c_board_bus);
}

/* ==========================================================================
 * I2C DMA TX / RX Interrupt Handlers
 * ========================================================================== */

void BOARD_I2C_DMA_TX_ISR(void) {
    i2c_dma_tx_isr(&i2c_board_bus);
}

void BOARD_I2C_DMA_RX_ISR(void) {
    i2c_dma_rx_isr(&i2c_board_bus);
}

/* ==========================================================================
 * UART Interrupt Handlers
 * ========================================================================== */

#if CONSOLE_USB
void BOARD_UART_ISR(void) {
    uart_isr(UART_PORT_USB);
}
#endif // CONSOLE_USB

#if CONSOLE_BT
void BOARD_BT_UART_ISR(void) {
    uart_isr(UART_PORT_BT);
}
#endif // CONSOLE_BT
