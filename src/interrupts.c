/**
 * @file interrupts.c
 * @brief Interrupt Service Routines (ISRs)
 * 
 * Contains ISR implementations for DMA and I2C peripherals.
 * These override the weak default handlers from libopencm3.
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "drivers/i2c.h"

/* ==========================================================================
 * External I2C Control Reference
 * ========================================================================== */

/* The I2C control structure is defined in mpu6050.c */
extern I2C_Control_t i2c;

/* ==========================================================================
 * I2C1 Event Interrupt Handler
 * ========================================================================== */

void i2c1_ev_isr(void) {
    i2c_ev_isr(&i2c);
}

/* ==========================================================================
 * I2C1 Error Interrupt Handler
 * ========================================================================== */

void i2c1_er_isr(void) {
    i2c_er_isr(&i2c);
}

/* ==========================================================================
 * DMA1 Channel 6 (I2C1 TX) Interrupt Handler
 * ========================================================================== */

void dma1_channel6_isr(void) {
    i2c_dma_tx_isr(&i2c);
}

/* ==========================================================================
 * DMA1 Channel 7 (I2C1 RX) Interrupt Handler
 * ========================================================================== */

void dma1_channel7_isr(void) {
    i2c_dma_rx_isr(&i2c);
}
