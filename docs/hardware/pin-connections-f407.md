# Hiwonder ROS Robot Control Board (STM32F407VET6) Pin Connections

Pin assignments come from the vendor firmware project
(`RosRobotControllerM4.ioc` and `Hiwonder/Portings/motor_porting.c` in
`RosRobotControllerM4_mecanum_8V.zip`) and the
[Hiwonder wiki](https://wiki.hiwonder.com/projects/ROS-Robot-Control-Board/en/latest/docs/1_Controller_Hardware_Course.html).

**MCU:** STM32F407VET6 (Cortex-M4F, 168 MHz, 512K flash, 128K SRAM + 64K CCM)
**Clock:** 16 MHz HSE crystal (from the vendor firmware's PLL setup)
**Supply:** DC 5V – 12.6V

## Used by this firmware

| Function | Pins | Peripheral | Notes |
|----------|------|------------|-------|
| User LED | PE10 | GPIO | Active low; heartbeat |
| Console | PD8 (TX), PD9 (RX) | USART3 | Type-C USB-serial (CH9102), found from the vendor firmware. Not a ROM bootloader port (those are USART1 PA9/PA10 and USART3 on PB10/PB11 or PC10/PC11): flash over SWD |
| Bluetooth console | PD5 (TX → module RXD), PD6 (RX ← module TXD) | USART2 | HC-05/HC-06 at `BT_BAUDRATE`; check the header's supply voltage before connecting |
| MPU-6050 | PB10 (SCL), PB11 (SDA) | I2C2 | DMA1 stream 7 (TX) / stream 2 (RX), channel 7 |
| motor1 | Port M1 | see below | `BOARD_MOTOR1_PORT` in `src/board/f407/board_config.h` |
| motor2 | Port M2 | see below | `BOARD_MOTOR2_PORT` |
| SWD | PA13 (SWDIO), PA14 (SWCLK) | | ST-Link |

## Encoder motor ports

Each port's driver has two PWM inputs: PWM on the forward input with the reverse
input low turns the motor forward (vendor convention), and vice versa. The
encoders are quadrature, read by a timer in encoder mode.

| Port | Forward PWM | Reverse PWM | Encoder A / B |
|------|-------------|-------------|---------------|
| M1 | TIM1_CH4 PE14 | TIM1_CH3 PE13 | TIM5 PA0 / PA1 |
| M2 | TIM1_CH2 PE11 | TIM1_CH1 PE9 | TIM2 PA15 / PB3 |
| M3 | TIM9_CH1 PE5 | TIM9_CH2 PE6 | TIM4 PB6 / PB7 |
| M4 | TIM11_CH1 PB9 | TIM10_CH1 PB8 | TIM3 PB4 / PB5 |

## Not used yet

| Function | Pins | Notes |
|----------|------|-------|
| MPU-6050 interrupt | PB12 | EXTI12 |
| Motor enable sense | PD3 | Input |
| Battery voltage | PB0 | ADC IN8 |
| Buzzer | PA8 | |
| User buttons | PE0 (KEY2), PE1 (KEY1) | Active low |
| PWM servos | PA11, PA12, PC8, PC9 | GPIO outputs in vendor firmware |
| Serial bus servo | PC6 (TX), PC7 (RX), PE7 (TX enable), PE8 (RX enable) | USART6 |
| SBUS receiver | PD2 | UART5 RX, inverted through NPN transistor |
| Host link | PD8 (TX), PD9 (RX) | USART3, 1 Mbit/s in vendor firmware |
| USB host | PB14 (D+), PB15 (D-) | |
