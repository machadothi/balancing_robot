# Balancing Robot

A self-balancing robot project using STM32 Blue Pill (STM32F103C8T6) with FreeRTOS. The robot uses an MPU6050 IMU with a Kalman filter for stable angle estimation.

![Balancing Robot](img/balancing_robot.png)

## Features

- **FreeRTOS** - Real-time operating system for task management
- **MPU6050 IMU** - 6-axis accelerometer and gyroscope
- **Kalman Filter** - Sensor fusion for accurate angle estimation
- **TB6612 Motor Driver** - Dual H-bridge motor control
- **UART Communication** - Serial interface at 921600 baud

## Hardware

- STM32F103C8T6 (Blue Pill)
- MPU6050 IMU module
- TB6612FNG motor driver
- DC motors with encoders
- USB-Serial adapter (for debugging)

## Project Structure

```
balancing-robot/
├── src/
│   ├── main.c              # Main application
│   ├── communication/      # I2C and UART drivers
│   ├── imu/                # MPU6050 driver and Kalman filter
│   ├── motor/              # Motor control
│   ├── robot/              # Robot task and control logic
│   ├── log/                # Logging utilities
│   └── rtos/               # FreeRTOS source files
├── test/
│   └── statistics.py       # Kalman filter analysis script
├── img/                    # Images and plots
└── README.md
```

## Building

See [BUILD.md](BUILD.md) for detailed build instructions.

### Quick Start

```bash
# Clone the repository
git clone --recurse-submodules https://github.com/machadothi/stm32f103c8t6.git
cd stm32f103c8t6/rtos/balancing-robot/src

# Build
make

# Flash to STM32
make flash
```

## Kalman Filter Performance

A 1D Kalman filter is implemented to fuse accelerometer and gyroscope data for stable angle estimation. The filter significantly reduces measurement noise while maintaining fast response to actual movement.

### Filter Analysis

![Kalman Filter Analysis](img/kalman_filter_analysis.png)

The analysis shows:
- **Time series comparison** - Raw accelerometer vs filtered Kalman output
- **Distribution comparison** - Tighter spread of Kalman filtered values
- **Noise reduction metrics** - Standard deviation and range improvements

### Running the Analysis

```bash
cd test/
python3 statistics.py
```

This will collect 1000 samples from the IMU and generate the performance analysis plot.

## Configuration

### Kalman Filter Tuning

In `src/robot/robot.c`:
- `Q_angle` - Process noise (higher = faster response, more noise)
- `R_measure` - Measurement noise (higher = smoother, slower response)

### Gyro Calibration

In `src/imu/mpu6050.h`:
- `GYRO_CONST_ERROR_MEAS` - Set to negative of gyro reading when stationary

## License

See [LICENSE](LICENSE) for details.
