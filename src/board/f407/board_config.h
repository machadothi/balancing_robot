/**
 * @file board_config.h
 * @brief Hiwonder ROS Robot Control Board (STM32F407VET6) peripheral assignments
 *
 * Taken from the vendor CubeMX project (RosRobotControllerM4.ioc), see
 * docs/hardware/pin-connections-f407.md. Motor port pins live in
 * src/motor/motor_hiwonder.c.
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Console: USART1 on the Type-C USB-serial port (PA9 = TX, PA10 = RX) */
#define BOARD_UART               USART1
#define BOARD_UART_RCC           RCC_USART1
#define BOARD_UART_IRQ           NVIC_USART1_IRQ
#define BOARD_UART_ISR           usart1_isr
#define BOARD_UART_PORT          GPIOA
#define BOARD_UART_PORT_RCC      RCC_GPIOA
#define BOARD_UART_TX_PIN        GPIO9
#define BOARD_UART_RX_PIN        GPIO10
#define BOARD_UART_AF            GPIO_AF7

/* On-board MPU-6050: I2C2 (PB10 = SCL, PB11 = SDA) */
#define BOARD_I2C                I2C2
#define BOARD_I2C_RCC            RCC_I2C2
#define BOARD_I2C_PORT           GPIOB
#define BOARD_I2C_PORT_RCC       RCC_GPIOB
#define BOARD_I2C_SCL_PIN        GPIO10
#define BOARD_I2C_SDA_PIN        GPIO11
#define BOARD_I2C_AF             GPIO_AF4
#define BOARD_I2C_EV_IRQ         NVIC_I2C2_EV_IRQ
#define BOARD_I2C_ER_IRQ         NVIC_I2C2_ER_IRQ
#define BOARD_I2C_EV_ISR         i2c2_ev_isr
#define BOARD_I2C_ER_ISR         i2c2_er_isr

/* I2C2_TX = DMA1 stream 7, I2C2_RX = DMA1 stream 2, both channel 7 */
#define BOARD_I2C_DMA            DMA1
#define BOARD_I2C_DMA_RCC        RCC_DMA1
#define BOARD_I2C_DMA_CHANNEL    DMA_SxCR_CHSEL_7
#define BOARD_I2C_DMA_TX         DMA_STREAM7
#define BOARD_I2C_DMA_RX         DMA_STREAM2
#define BOARD_I2C_DMA_TX_IRQ     NVIC_DMA1_STREAM7_IRQ
#define BOARD_I2C_DMA_RX_IRQ     NVIC_DMA1_STREAM2_IRQ
#define BOARD_I2C_DMA_TX_ISR     dma1_stream7_isr
#define BOARD_I2C_DMA_RX_ISR     dma1_stream2_isr

/* Motor ports (M1-M4 on the silkscreen) used as motor1 and motor2 */
#define BOARD_MOTOR1_PORT        1
#define BOARD_MOTOR2_PORT        2

#endif // BOARD_CONFIG_H
