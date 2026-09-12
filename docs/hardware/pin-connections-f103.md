# STM32F103C8T6 Pin Connections

Complete pin mapping for the self-balancing robot project.

## Table of Contents

- [Overview](#overview)
- [MPU6050 IMU (I2C)](#mpu6050-imu-i2c)
- [TB6612FNG Motor Driver](#tb6612fng-motor-driver)
- [A4988 Stepper Driver (Optional)](#a4988-stepper-driver-optional)
- [UART Debug Interface](#uart-debug-interface)
- [LED Indicators](#led-indicators)
- [Motor Encoders](#motor-encoders)
- [Power Connections](#power-connections)
- [Pin Summary Table](#pin-summary-table)

---

## Overview

This document describes all hardware connections between the STM32F103C8T6 (Blue Pill) and peripheral modules used in the balancing robot project.

**System Clock:** 72 MHz  
**Voltage Level:** 3.3V logic

---

## MPU6050 IMU (I2C)

The MPU6050 6-axis accelerometer/gyroscope connects via I2C1.

| MPU6050 Pin | STM32 Pin | Function | Notes |
|------------|-----------|----------|-------|
| VCC | 3.3V | Power | **Use 3.3V, NOT 5V** |
| GND | GND | Ground | |
| SCL | **PB6** | I2C1 Clock | 100 kHz standard mode |
| SDA | **PB7** | I2C1 Data | |
| XDA | - | Not connected | Auxiliary I2C (unused) |
| XCL | - | Not connected | Auxiliary I2C (unused) |
| AD0 | GND | I2C Address Select | GND = 0x68, VCC = 0x69 |
| INT | - | Not connected | Interrupt output (optional) |

**I2C Configuration:**
- **Bus Speed:** 100 kHz
- **Slave Address:** 0x68 (when AD0 = GND)
- **DMA Enabled:** DMA1 Channel 6 (TX), DMA1 Channel 7 (RX)
- **Pull-ups:** External 4.7kΩ resistors on SCL/SDA recommended

**GPIO Enable Pin (Optional):**
- **PA10** - Used in code to enable MPU6050 power (application-specific)

---

## TB6612FNG Motor Driver

Dual DC motor driver for controlling two JGA25-370 brushed motors.

### Control Pins

| TB6612 Pin | STM32 Pin | Function | Description |
|-----------|-----------|----------|-------------|
| VM | 7-12V | Motor Power | Connect to battery/power supply |
| VCC | 3.3V | Logic Power | Connect to STM32 3.3V |
| GND | GND | Ground | Common ground (connect to both STM32 and battery) |
| STBY | **PB4** | Standby Control | HIGH = Active, LOW = Standby |

### Motor A Connections (Left Motor)

| TB6612 Pin | STM32 Pin | Function | Timer/Channel |
|-----------|-----------|----------|---------------|
| AIN1 | **PB3** | Direction Input 1 | GPIO Output |
| AIN2 | **PA8** | Direction Input 2 | GPIO Output |
| PWMA | **PB1** | Speed Control | TIM3_CH4 |
| AO1 | - | Motor A Terminal | JGA25-370 Red wire (+) |
| AO2 | - | Motor A Terminal | JGA25-370 Black wire (-) |

### Motor B Connections (Right Motor)

| TB6612 Pin | STM32 Pin | Function | Timer/Channel |
|-----------|-----------|----------|---------------|
| BIN1 | **PB5** | Direction Input 1 | GPIO Output |
| BIN2 | **PB8** | Direction Input 2 | GPIO Output |
| PWMB | **PB0** | Speed Control | TIM3_CH3 |
| BO1 | - | Motor B Terminal | JGA25-370 Red wire (+) |
| BO2 | - | Motor B Terminal | JGA25-370 Black wire (-) |

### Direction Truth Table

| IN1 | IN2 | Motor State |
|-----|-----|-------------|
| HIGH | LOW | Forward (CW) |
| LOW | HIGH | Reverse (CCW) |
| HIGH | HIGH | Short Brake |
| LOW | LOW | Coast (Free Run) |

**PWM Configuration:**
- **Frequency:** 1 kHz (optimized for JGA25-370 motors)
- **Resolution:** 1000 steps (0.1% duty cycle)
- **Timer:** TIM3 (APB1)
- **Max Speed:** 255 (0-100% mapped to 0-255)

**Motor Specifications (JGA25-370):**
- **Rated Voltage:** 6V or 12V (check your model)
- **No-load Current:** ~100mA
- **Rated Current:** ~200-300mA
- **Stall Current:** ~2A
- **TB6612 Max Current:** 1.2A continuous per channel

---

## A4988 Stepper Driver (Optional)

If using A4988 stepper motor driver instead of TB6612FNG for DC motors.

### Stepper Motor A (or use as DC motor control)

| A4988 Pin | STM32 Pin | Function | Notes |
|----------|-----------|----------|-------|
| VDD | 3.3V | Logic Power | |
| GND | GND | Ground | |
| VMOT | 8-35V | Motor Power | Connect to power supply |
| GND | GND | Power Ground | |
| ENABLE | **PB4** | Enable (Active LOW) | LOW = Enabled |
| MS1 | GND | Microstep Select 1 | Full step mode |
| MS2 | GND | Microstep Select 2 | Full step mode |
| MS3 | GND | Microstep Select 3 | Full step mode |
| RESET | 3.3V | Reset (Active LOW) | Tie HIGH |
| SLEEP | 3.3V | Sleep (Active LOW) | Tie HIGH |
| STEP | **PB1** | Step Pulse | Connect to TIM3_CH4 PWM |
| DIR | **PB3** | Direction | GPIO Output |
| 1A, 1B | - | Motor Coil A | Connect to stepper motor |
| 2A, 2B | - | Motor Coil B | Connect to stepper motor |

### Stepper Motor B

| A4988 Pin | STM32 Pin | Function | Notes |
|----------|-----------|----------|-------|
| VDD | 3.3V | Logic Power | |
| GND | GND | Ground | |
| ENABLE | **PB4** | Enable (Active LOW) | Shared with Motor A |
| STEP | **PB0** | Step Pulse | Connect to TIM3_CH3 PWM |
| DIR | **PB5** | Direction | GPIO Output |
| 1A, 1B | - | Motor Coil A | Connect to stepper motor |
| 2A, 2B | - | Motor Coil B | Connect to stepper motor |

**Note:** The current firmware is configured for TB6612FNG. To use A4988, modify the motor driver initialization and control logic.

---

## UART Debug Interface

Serial communication for debugging and AT command interface.

| Function | STM32 Pin | Alt Function | Notes |
|----------|-----------|--------------|-------|
| TX | **PA2** | USART2_TX | Transmit to USB-Serial adapter |
| RX | **PA3** | USART2_RX | Receive from USB-Serial adapter |

**UART Configuration:**
- **Peripheral:** USART2
- **Baud Rate:** 921600 bps
- **Data Bits:** 8
- **Stop Bits:** 1
- **Parity:** None
- **Flow Control:** None
- **Mode:** Full-duplex (TX + RX with interrupts)

**USB-Serial Adapter Connections:**
- Connect STM32 TX → Adapter RX
- Connect STM32 RX → Adapter TX
- Connect GND → Adapter GND
- **Do NOT connect VCC** (power STM32 separately)

---

## LED Indicators

Visual feedback LEDs for system status.

| LED | STM32 Pin | Function | Notes |
|-----|-----------|----------|-------|
| Onboard LED | **PC13** | Heartbeat | Blue Pill built-in LED (Active LOW) |
| RGB Red | **PB14** | Status/Error | External RGB LED |
| RGB Green | **PB12** | Status/OK | External RGB LED |
| RGB Blue | **PB13** | Status/Info | External RGB LED |

**LED Configuration:**
- **Mode:** Push-pull output, 2 MHz
- **Current Limiting:** Use 220Ω-470Ω series resistors
- **Common Anode/Cathode:** Configure based on your RGB LED type

---

## Motor Encoders

Optional encoder inputs for closed-loop motor control and odometry.

| Encoder | STM32 Pin | Function | Interrupt |
|---------|-----------|----------|-----------|
| Motor A | **PA5** | Pulse Input | EXTI5 (Rising Edge) |
| Motor B | **PA6** | Pulse Input | EXTI6 (Rising Edge) |

**Encoder Configuration:**
- **Input Mode:** Floating input with external pull-up
- **Trigger:** Rising edge
- **Interrupt:** EXTI9_5_IRQ (shared for EXTI5-9)
- **Priority:** 0x80 (medium priority)
- **Function:** Pulse counting for speed measurement

**Typical Encoder Specs:**
- Type: Hall effect or optical encoder
- Output: Open collector or push-pull
- Resolution: 11-20 pulses per revolution (typical)

---

## Power Connections

### STM32 Power

| Pin | Connection | Voltage | Notes |
|-----|-----------|---------|-------|
| VDD (3.3V) | LDO Regulator Output | 3.3V | Multiple pins, connect all |
| GND | Common Ground | 0V | Multiple pins, connect all |
| VBAT | 3.3V or Battery | 3.3V | RTC backup (optional) |
| VDD_A | 3.3V | 3.3V | Analog power |
| VSS_A | GND | 0V | Analog ground |

### Motor Power

| Component | Voltage | Current | Notes |
|----------|---------|---------|-------|
| TB6612FNG VM | 7-12V | Up to 1.2A per motor | Use adequate power supply |
| Motors | Rated voltage | 0.5-1A each | Check motor specifications |
| STM32 VDD | 3.3V | ~100mA | From onboard LDO or external regulator |
| MPU6050 | 3.3V | ~4mA | Low power consumption |

**Power Supply Recommendations:**
- **Motor Power:** 2S LiPo (7.4V) or 9V battery
- **Logic Power:** Use Blue Pill's onboard 3.3V LDO or separate buck converter
- **Capacitors:** Add 100µF electrolytic near motor driver VM pin
- **Decoupling:** 0.1µF ceramic caps near all IC VDD pins

---

## Pin Summary Table

Complete pin assignment reference.

| STM32 Pin | Function | Peripheral | Module | Direction |
|-----------|----------|------------|---------|-----------|
| **PA2** | UART TX | USART2_TX | Debug | Output |
| **PA3** | UART RX | USART2_RX | Debug | Input |
| **PA5** | Encoder A | EXTI5 | Motor A | Input |
| **PA6** | Encoder B | EXTI6 | Motor B | Input |
| **PA8** | Motor A IN2 | GPIO | TB6612 | Output |
| **PA10** | IMU Enable | GPIO | MPU6050 | Output |
| | | | | |
| **PB0** | Motor B PWM | TIM3_CH3 | TB6612 | PWM Out |
| **PB1** | Motor A PWM | TIM3_CH4 | TB6612 | PWM Out |
| **PB3** | Motor A IN1 | GPIO | TB6612 | Output |
| **PB4** | Standby | GPIO | TB6612 | Output |
| **PB5** | Motor B IN1 | GPIO | TB6612 | Output |
| **PB6** | I2C SCL | I2C1_SCL | MPU6050 | I2C |
| **PB7** | I2C SDA | I2C1_SDA | MPU6050 | I2C |
| **PB8** | Motor B IN2 | GPIO | TB6612 | Output |
| **PB12** | LED Green | GPIO | Status LED | Output |
| **PB13** | LED Blue | GPIO | Status LED | Output |
| **PB14** | LED Red | GPIO | Status LED | Output |
| | | | | |
| **PC13** | Onboard LED | GPIO | Heartbeat | Output |

---

## Complete System Wiring Diagram

### Overview Diagram

```
                Power Supply (7-12V Battery)
                      │
                      ├── VM ──────────┐
                      │                │
                      └── GND ─────────┼──────────┐
                                       │          │
    STM32F103C8T6 (Blue Pill)          │   TB6612 Motor Driver
                                       │
    3.3V ──────────────────────────────┼── VCC
    GND ───────────────────────────────┼── GND
                                       │
    PB4 ───────────────────────────────┼── STBY   (Enable, HIGH = active)
                                       │
    PB3 ───────────────────────────────┼── AIN1   (Motor A direction)
    PA8 ───────────────────────────────┼── AIN2   (Motor A direction)
    PB1 (TIM3_CH4) ────────────────────┼── PWMA   (Motor A speed)
                                       │
    PB5 ───────────────────────────────┼── BIN1   (Motor B direction)
    PB8 ───────────────────────────────┼── BIN2   (Motor B direction)
    PB0 (TIM3_CH3) ────────────────────┼── PWMB   (Motor B speed)
                                       │
                                       │── AO1 ──── JGA25-370 Motor A [Red]
                                       │── AO2 ──── JGA25-370 Motor A [Black]
                                       │
                                       │── BO1 ──── JGA25-370 Motor B [Red]
                                       └── BO2 ──── JGA25-370 Motor B [Black]
```

### Detailed Connection Diagram

```
                    STM32F103C8T6 (Blue Pill)
                   ┌─────────────────────┐
                   │                     │
        ┌──────────┤ PB6 (I2C1_SCL)     │
        │   ┌──────┤ PB7 (I2C1_SDA)     │
        │   │      │                     │
        │   │  ┌───┤ PA2 (USART2_TX)    │──────> USB-Serial RX
        │   │  │ ┌─┤ PA3 (USART2_RX)    │<────── USB-Serial TX
        │   │  │ │ │                     │
        │   │  │ │ │ PB0 (TIM3_CH3) ├───┤──────> TB6612 PWMB
        │   │  │ │ │ PB1 (TIM3_CH4) ├───┤──────> TB6612 PWMA
        │   │  │ │ │ PB3 ├──────────────┤──────> TB6612 AIN1
        │   │  │ │ │ PB4 ├──────────────┤──────> TB6612 STBY
        │   │  │ │ │ PB5 ├──────────────┤──────> TB6612 BIN1
        │   │  │ │ │ PB8 ├──────────────┤──────> TB6612 BIN2
        │   │  │ │ │ PA8 ├──────────────┤──────> TB6612 AIN2
        │   │  │ │ │                     │
        │   │  │ │ │ PA5 <──────────────┤<────── Motor A Encoder
        │   │  │ │ │ PA6 <──────────────┤<────── Motor B Encoder
        │   │  │ │ │                     │
        │   │  │ │ │ PC13 ├─────────────┤──────> Onboard LED
        │   │  │ │ │ PB12 ├─────────────┤──────> RGB Green
        │   │  │ │ │ PB13 ├─────────────┤──────> RGB Blue
        │   │  │ │ │ PB14 ├─────────────┤──────> RGB Red
        │   │  │ │ │                     │
        │   │  │ │ │ 3.3V ├──┬───────┬──┤
        │   │  │ │ │ GND  ├──┼───┬───┼──┤
        │   │  │ │ └──────┼──┼───┼───┼──┘
        │   │  │ │        │  │   │   │
        │   │  │ │        │  │   │   │
        │   │  │ └────────┼──┘   │   │
        │   │  └─────────┐│      │   │
        │   │            ││      │   │
        │   │    MPU6050 ││      │   │
        │   │   ┌────────┴┴───┐  │   │
        │   └───┤ SCL      VCC├──┘   │
        └───────┤ SDA      GND├──────┘
                └─────────────┘

          TB6612FNG Motor Driver               Battery Pack
         ┌──────────────────────┐            ┌──────────┐
    PB4──┤ STBY            VM   ├────────────┤ + (7-12V)│
    PB3──┤ AIN1            VCC  ├──┐         │          │
    PA8──┤ AIN2            GND  ├──┼─────────┤ - (GND)  │
    PB1──┤ PWMA            AO1  ├──┼─────┐   └──────────┘
         │                 AO2  ├──┼───┐ │
    PB5──┤ BIN1            BO1  ├──┼─┐ │ │   [100µF Cap]
    PB8──┤ BIN2            BO2  ├──┼─┼─┼─┼───near VM/GND
    PB0──┤ PWMB                 │  │ │ │ │
         └──────────────────────┘  │ │ │ │
              │                    │ │ │ │
         STM32 3.3V ───────────────┘ │ │ │
         STM32 GND ───────────────────┘ │ │
                                        │ │
              JGA25-370 Motor A         │ │
             ┌──────────────────┐       │ │
             │  [Geared Motor]  │       │ │
       Red ──┤ +            Enc ├───────┼─┼──> PA5
      Black─┤ -                │       │ │
             └──────────────────┘       │ │
                                        │ │
              JGA25-370 Motor B         │ │
             ┌──────────────────┐       │ │
             │  [Geared Motor]  │       │ │
       Red ──┤ +            Enc ├───────┼─┘──> PA6
      Black─┤ -                │       │
             └──────────────────┘       │
                   │                    │
                   └────────────────────┘
```

### Power Distribution Diagram

```
        Battery (7-12V, 2S/3S LiPo recommended)
                      │
                      │
        ┌─────────────┴─────────────┐
        │                           │
        │  [100µF Capacitor]        │
        │   across VM/GND           │
        │                           │
    ┌───┴───┐                       │
    │  VM   │                       │
    │       │ TB6612FNG             │
    │  GND  │                       │
    └───┬───┘                       │
        │                           │
        │                      ┌────┴────┐
        └──────────────────────┤   GND   │
                               │ STM32   │
                               │ 3.3V ───┼──> VCC (TB6612)
                               └─────────┘
                               
    WARNING: Do NOT connect battery voltage directly to STM32!
    The STM32 runs on 3.3V only.
```

---

## Wiring Best Practices

### Critical Connections

1. **Common Ground:** 
   - STM32 GND, TB6612 GND, and Battery GND **MUST** be connected together
   - Use thick wires (22-20 AWG) for ground connections

2. **Power Isolation:**
   - **NEVER** connect battery voltage (7-12V) directly to STM32
   - STM32 runs on **3.3V only** - use onboard regulator or separate buck converter
   - TB6612 VCC connects to STM32 3.3V (logic power)
   - TB6612 VM connects to battery voltage (motor power)

3. **Decoupling Capacitors:**
   - Add **100µF electrolytic capacitor** across TB6612 VM and GND pins (as close as possible)
   - Add **0.1µF ceramic capacitors** near all IC VDD pins
   - Prevents voltage spikes from motors

4. **Wire Gauge Recommendations:**
   - Motor power (VM): 20-22 AWG
   - Motor outputs: 22-24 AWG
   - Control signals: 26-28 AWG
   - Ground: 20-22 AWG (thick)

### Connection Order

**Always follow this sequence to avoid damage:**

1. ✅ Connect all ground wires first (STM32, TB6612, Battery)
2. ✅ Connect 3.3V from STM32 to TB6612 VCC
3. ✅ Connect control signals (PB0-PB8, PA8)
4. ✅ Connect motors to TB6612 outputs (AO1/AO2, BO1/BO2)
5. ✅ Add capacitor across TB6612 VM/GND
6. ✅ **LAST:** Connect battery to TB6612 VM

**Disconnection:** Reverse order (battery first, then signals)

## Notes

1. **Voltage Levels:** STM32F103 is **NOT 5V tolerant** on all pins. Always use 3.3V logic levels.

2. **Current Limitations:** 
   - GPIO pins: 25mA max per pin
   - TB6612 output: 1.2A continuous, 3.2A peak per channel
   - JGA25-370 motors: ~300mA rated, ~2A stall

3. **Pull-ups:** I2C lines require external 4.7kΩ pull-up resistors to 3.3V.

4. **Motor Driver:** This project uses TB6612FNG for JGA25-370 DC motors at 1kHz PWM.

5. **DMA Channels:** I2C1 uses DMA1 CH6 (TX) and DMA1 CH7 (RX) for non-blocking transfers.

6. **Timer Conflicts:** TIM3 is dedicated to motor PWM. Do not use TIM3 for other purposes.

7. **EXTI Conflicts:** EXTI5 and EXTI6 are used for motor encoders. Avoid using other pins with these EXTI lines.

8. **Power Sequencing:** Power the STM32 before connecting motors to prevent GPIO floating states from triggering unwanted motor movement.

9. **Motor Direction:** If a motor spins backward, either:
   - Swap the motor wires at TB6612 output (AO1↔AO2 or BO1↔BO2), or
   - Invert direction in software

10. **Heat Dissipation:** TB6612 may get warm under load. Ensure adequate ventilation or add heatsink.

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-01 | Auto-generated | Initial pin connection documentation |

---

**For more information:**
- See [02 — Build and Configuration](../02-build-and-configuration.md) for build instructions
- See [10 — AT Commands](../10-at-commands.md) for the command interface reference
- See the [documentation index](../README.md) for the full book
