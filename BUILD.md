# Build Instructions

This project is self-contained with all dependencies managed as git submodules.

## Quick Start

The easiest way to build is using the setup script:

```bash
git clone https://github.com/machadothi/balancing-robot.git
cd balancing-robot
./scripts/setup.sh
```

This will:
1. Check for required tools
2. Clone libopencm3 and FreeRTOS-Kernel as submodules
3. Build libopencm3 for STM32F1
4. Build the project using CMake

## Prerequisites

### ARM Toolchain

**Ubuntu/Debian:**
```bash
sudo apt install gcc-arm-none-eabi cmake make git
```

**Arch Linux:**
```bash
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib cmake make git
```

**Manual Installation:**
Download from [ARM Developer](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads):
```bash
cd /opt
sudo tar xjf ~/Downloads/gcc-arm-none-eabi-*-linux.tar.bz2
sudo mv gcc-arm-none-eabi-* gcc-arm
export PATH="/opt/gcc-arm/bin:$PATH"
```

Verify:
```bash
arm-none-eabi-gcc --version
```

### ST-Link Tools (for flashing)

```bash
# Ubuntu/Debian
sudo apt install stlink-tools

# Or build from source
git clone https://github.com/stlink-org/stlink.git
cd stlink && cmake . && make && sudo make install
```

## Manual Build

If you prefer to build manually:

### 1. Clone with Submodules

```bash
git clone --recurse-submodules https://github.com/machadothi/balancing-robot.git
cd balancing-robot
```

Or if already cloned:
```bash
git submodule update --init --recursive
```

### 2. Build libopencm3

```bash
cd lib/libopencm3
make TARGETS=stm32/f1 -j$(nproc)
cd ../..
```

### 3. Build with CMake

```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
make -j$(nproc)
```

## Build Outputs

After building, these files are in the `build/` directory:
- `balancing-robot.elf` - ELF executable (for debugging)
- `balancing-robot.bin` - Binary for flashing
- `balancing-robot.hex` - Intel HEX format
- `balancing-robot.map` - Memory map

## Flashing

Connect your ST-Link programmer to the Blue Pill and run:

```bash
cd build
make flash
```

Or manually:
```bash
st-flash write build/balancing-robot.bin 0x8000000
```

## Serial Monitor

The project outputs debug data via UART2 (PA2) at 921600 baud:

```bash
picocom -b 921600 /dev/ttyUSB0
```

## Clean Builds

```bash
# Clean CMake build
rm -rf build

# Clean and rebuild everything
./scripts/setup.sh --clean

# Just rebuild libopencm3 and project
./scripts/setup.sh --rebuild
```

## Project Structure

```
balancing-robot/
├── CMakeLists.txt          # Main CMake configuration
├── cmake/
│   └── arm-none-eabi.cmake # ARM toolchain file
├── lib/
│   ├── libopencm3/         # ARM Cortex-M library (submodule)
│   └── FreeRTOS-Kernel/    # RTOS kernel (submodule)
├── scripts/
│   └── setup.sh            # Project setup script
├── src/
│   ├── main.c
│   ├── config.h            # Centralized configuration
│   ├── FreeRTOSConfig.h    # FreeRTOS configuration
│   ├── stm32f103c8t6.ld    # Linker script
│   ├── communication/      # I2C, UART drivers
│   ├── filter/             # Kalman, Complementary filters
│   ├── imu/                # MPU6050 driver
│   ├── led/                # LED control
│   ├── log/                # Logging module
│   ├── motor/              # Motor control
│   ├── robot/              # Robot control task
│   └── rtos/               # FreeRTOS integration
├── test/
│   └── statistics.py       # Filter analysis script
└── img/                    # Images and plots
```

## Troubleshooting

### Build Errors

- Run `./scripts/setup.sh --clean` to start fresh
- Check ST-Link connection
- Verify device is detected: `st-info --probe`
- Try resetting the board while flashing

### No Serial Output

- Check UART wiring (PA2 is TX)
- Verify baud rate matches (921600)
- Ensure USB-Serial adapter is working: `ls /dev/ttyUSB*`
