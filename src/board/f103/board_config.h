/**
 * @file board_config.h
 * @brief Blue Pill (STM32F103C8T6) peripheral assignments
 *
 * Wiring is documented in docs/hardware/pin-connections-f103.md. Motor pins live in
 * src/motor/motor.h (TB6612FNG driver).
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Console: USART2 (PA2 = TX, PA3 = RX) */
#define BOARD_UART               USART2
#define BOARD_UART_RCC           RCC_USART2
#define BOARD_UART_IRQ           NVIC_USART2_IRQ
#define BOARD_UART_ISR           usart2_isr
#define BOARD_UART_PORT          GPIOA
#define BOARD_UART_PORT_RCC      RCC_GPIOA
#define BOARD_UART_TX_PIN        GPIO2
#define BOARD_UART_RX_PIN        GPIO3
#define BOARD_UART_AF            0

/* MPU6050: I2C1 (PB6 = SCL, PB7 = SDA) */
#define BOARD_I2C                I2C1
#define BOARD_I2C_RCC            RCC_I2C1
#define BOARD_I2C_PORT           GPIOB
#define BOARD_I2C_PORT_RCC       RCC_GPIOB
#define BOARD_I2C_SCL_PIN        GPIO6
#define BOARD_I2C_SDA_PIN        GPIO7
#define BOARD_I2C_AF             0
#define BOARD_I2C_EV_IRQ         NVIC_I2C1_EV_IRQ
#define BOARD_I2C_ER_IRQ         NVIC_I2C1_ER_IRQ
#define BOARD_I2C_EV_ISR         i2c1_ev_isr
#define BOARD_I2C_ER_ISR         i2c1_er_isr

#define BOARD_I2C_DMA            DMA1
#define BOARD_I2C_DMA_RCC        RCC_DMA1
#define BOARD_I2C_DMA_TX         DMA_CHANNEL6
#define BOARD_I2C_DMA_RX         DMA_CHANNEL7
#define BOARD_I2C_DMA_TX_IRQ     NVIC_DMA1_CHANNEL6_IRQ
#define BOARD_I2C_DMA_RX_IRQ     NVIC_DMA1_CHANNEL7_IRQ
#define BOARD_I2C_DMA_TX_ISR     dma1_channel6_isr
#define BOARD_I2C_DMA_RX_ISR     dma1_channel7_isr

/* Switch on the MPU6050 VCC line, used for a hard reset */
#define BOARD_IMU_RESET_PORT     GPIOA
#define BOARD_IMU_RESET_PORT_RCC RCC_GPIOA
#define BOARD_IMU_RESET_PIN      GPIO10

#endif // BOARD_CONFIG_H
