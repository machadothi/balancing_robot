# Hiwonder ROS Robot Control Board (STM32F407VET6) Pin Connections

Pin facts collected from the
[Hiwonder wiki](https://wiki.hiwonder.com/projects/ROS-Robot-Control-Board/en/latest/docs/1_Controller_Hardware_Course.html).
The official schematic has not been published; entries marked **unverified**
still need confirming against the schematic, the vendor firmware, or the board itself.

**MCU:** STM32F407VET6 (Cortex-M4F, 168 MHz, 512K flash, 128K SRAM + 64K CCM)
**Clock:** 8 MHz HSE crystal
**Supply:** DC 5V – 12.6V

## Used by this firmware

| Function | Pin | Notes |
|----------|-----|-------|
| User LED | PE10 | Active low; heartbeat LED |
| SWDIO | PA13 | ST-Link |
| SWCLK | PA14 | ST-Link |

## Not used yet

| Function | Pins | Notes |
|----------|------|-------|
| MPU-6050 IMU | PB10 (SCL), PB11 (SDA) | I2C2, 10k pull-ups, INT pin not listed |
| Motor driver (YX-4055AM) | PE9, PE11, PE13, PE14, PE5, PE6, PB8, PB9 | PE9/11/13/14 = TIM1 CH1–4; PE5/PE6 = TIM9 CH1/2; PB8/PB9 = TIM4 CH3/4 (**unverified**) |
| Motor encoders | ? | Not documented; get from vendor firmware |
| UART1 | Type-C USB-serial | Programming (bootloader: DTR = reset, RTS = BOOT0) and console |
| UART2 | USB serial port 2 | Recommended link to Raspberry Pi / Jetson |
| Bus servo | PE7, PG6 (TX), PC7 (RX) | As listed in wiki (**unverified**: PG6 is not on a 100-pin package) |
| SBUS receiver | PD2 | Inverted through NPN transistor |
| Bluetooth | PD5, PD6 | USART2 TX/RX |
| OLED 0.96" (SPI) | PB13, PC3, PD14, PD13, PD12, PD11 | |
| USB host | PB14 (D+), PB15 (D-) | USB OTG HS in FS mode |
| Buzzer | PA4 | Through S8050 transistor |
| User buttons | PE0, PE1 | Active low |
| BOOT0 / BOOT1 | — | Pulled low (boot from flash) |
