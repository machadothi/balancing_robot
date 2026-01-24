# Build Instructions

## Prerequisites

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/machadothi/stm32f103c8t6.git
cd stm32f103c8t6
```

### 2. Install ARM Toolchain

Download the ARM GNU toolchain from [ARM Developer](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).

```bash
# Extract to /opt
cd /opt
sudo tar xjf ~/Downloads/gcc-arm-none-eabi-*-linux.tar.bz2
sudo mv gcc-arm-none-eabi-* gcc-arm

# Add to PATH (add to ~/.bashrc for persistence)
export PATH="/opt/gcc-arm/bin:$PATH"
```

Verify installation:
```bash
arm-none-eabi-gcc --version
```

### 3. Install st-flash (STLink Tools)

```bash
# Ubuntu/Debian
sudo apt install stlink-tools

# Or build from source
git clone https://github.com/stlink-org/stlink.git
cd stlink
cmake .
make
sudo make install
```

## Building the Project

```bash
cd stm32f103c8t6/rtos/balancing-robot/src

# Clean build
make clean

# Build
make
```

Build outputs are placed in the `build/` directory:
- `build/main.elf` - ELF executable
- `build/main.bin` - Binary for flashing
- `build/main.map` - Memory map

## Flashing

Connect your ST-Link programmer to the Blue Pill and run:

```bash
make flash
```

## Serial Monitor

The project outputs debug data via UART2 (PA2) at 921600 baud:

```bash
picocom -b 921600 /dev/ttyUSB0
```

## Testing

### Kalman Filter Analysis

```bash
cd ../test
pip install pyserial numpy matplotlib
python3 statistics.py
```

This collects IMU samples and generates a performance analysis plot saved to `img/kalman_filter_analysis.png`.

## Troubleshooting

### Build Errors

- Ensure submodules are initialized: `git submodule update --init --recursive`
- Check toolchain path: `which arm-none-eabi-gcc`

### Flash Errors

- Check ST-Link connection
- Verify device is detected: `st-info --probe`
- Try resetting the board while flashing

### No Serial Output

- Check UART wiring (PA2 is TX)
- Verify baud rate matches (921600)
- Ensure USB-Serial adapter is working: `ls /dev/ttyUSB*`
