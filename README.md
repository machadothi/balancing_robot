# Balancing Robot

A self-balancing robot project using STM32 with FreeRTOS. The robot uses an MPU6050 IMU with sensor fusion filters for stable angle estimation. It runs on the STM32 Blue Pill (STM32F103C8T6); a port to the Hiwonder ROS Robot Control Board (STM32F407VET6) is in progress.

![Balancing Robot](img/balancing_robot.png)

## Features

- **FreeRTOS** - Real-time operating system for task management
- **MPU6050 IMU** - 6-axis accelerometer and gyroscope
- **Dual Filter Implementation** - Both Kalman and Complementary filters available
- **TB6612 Motor Driver** - Dual H-bridge motor control
- **UART Communication** - Serial interface at 921600 baud

## Hardware

- STM32F103C8T6 (Blue Pill), or the Hiwonder ROS Robot Control Board (STM32F407VET6, see [docs/PIN_CONNECTIONS_F407.md](docs/PIN_CONNECTIONS_F407.md))
- MPU6050 IMU module
- TB6612FNG motor driver
- DC motors with encoders
- USB-Serial adapter (for debugging)

## Project Structure

```
balancing-robot/
├── CMakeLists.txt          # CMake build configuration
├── cmake/                  # CMake toolchain files
├── lib/
│   ├── libopencm3/         # ARM Cortex-M library (submodule)
│   └── FreeRTOS-Kernel/    # Real-time OS (submodule)
├── scripts/
│   └── setup.sh            # Project setup script
├── src/
│   ├── main.c              # Main application
│   ├── config.h            # Centralized configuration
│   ├── FreeRTOSConfig.h    # FreeRTOS configuration
│   ├── communication/      # I2C and UART drivers
│   ├── filter/             # Kalman and Complementary filters
│   ├── imu/                # MPU6050 driver and IMU interface
│   ├── led/                # LED control module
│   ├── motor/              # Motor control
│   ├── robot/              # Robot task and control logic
│   ├── log/                # Logging utilities
│   └── rtos/               # FreeRTOS integration
├── test/
│   └── statistics.py       # Filter analysis script
├── img/                    # Images and plots
└── README.md
```

## Building

See [BUILD.md](BUILD.md) for detailed build instructions.

### Quick Start

```bash
# Clone the repository
git clone https://github.com/machadothi/balancing-robot.git
cd balancing-robot

# Run setup script (clones dependencies, builds everything)
./scripts/setup.sh              # Blue Pill
BOARD=f407 ./scripts/setup.sh   # Hiwonder STM32F407 board

# Flash to STM32
cmake --build --preset f103 --target flash
```

### Manual Build

```bash
# Initialize submodules
git submodule update --init --recursive

# Build libopencm3
cd lib/libopencm3 && make TARGETS="stm32/f1 stm32/f4" && cd ../..

# Build with CMake (preset f103 or f407)
cmake --preset f103
cmake --build --preset f103
```

## Kalman Filter Performance

A 1D Kalman filter is implemented to fuse accelerometer and gyroscope data for stable angle estimation. The filter significantly reduces measurement noise while maintaining fast response to actual movement.

### Filter Analysis

![Kalman Filter Analysis](img/kalman_filter_analysis.png)

The analysis shows:
- **Time series comparison** - Raw accelerometer vs filtered Kalman output
- **Distribution comparison** - Tighter spread of Kalman filtered values
- **Noise reduction metrics** - Standard deviation and range improvements

## Sensor Fusion Filters

This project implements two sensor fusion filters for angle estimation: **Kalman Filter** and **Complementary Filter**.

### Why Two Filters?

Initially, the Kalman filter was implemented as the primary sensor fusion algorithm. While the Kalman filter is mathematically optimal under certain conditions, real-world testing revealed that the **Complementary filter provided better results** for this specific application.

### Filter Comparison

![Filter Comparison](img/filter_comparison.png)

The comparison graph shows both filters running simultaneously on the same IMU data. Key observations:

- **Complementary Filter** - Faster response, smoother output, less computational overhead
- **Kalman Filter** - More complex, requires tuning of Q and R parameters

The Complementary filter's simplicity and effectiveness made it the preferred choice for the balancing robot's real-time control loop.

### How the Filters Work

#### Kalman Filter
The Kalman filter uses a predict-update cycle:
1. **Predict** - Estimate angle using gyroscope integration
2. **Update** - Correct prediction using accelerometer measurement
3. Parameters: `Q_angle` (process noise), `R_measure` (measurement noise)

#### Complementary Filter
The Complementary filter combines high-pass (gyro) and low-pass (accel) filtering:
```
angle = α × (angle + gyro × dt) + (1 - α) × accel_angle
```
- `α = 0.96` - Trust 96% gyroscope, 4% accelerometer
- Simple, computationally efficient, and robust

### Switching Between Filters

In `src/config.h`, both filters can be configured. The robot task in `src/robot/robot.c` can easily switch between filters for comparison or choose the preferred one for production use.

### Running the Analysis

```bash
cd test/
python3 statistics.py
```

This will collect 1000 samples from the IMU and generate the performance analysis plot.

## Configuration

All tunable parameters are centralized in `src/config.h`:

### Filter Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `KALMAN_Q_ANGLE` | 0.1 | Kalman process noise covariance |
| `KALMAN_R_MEASURE` | 0.5 | Kalman measurement noise covariance |
| `COMPLEMENTARY_ALPHA` | 0.96 | Complementary filter weight (gyro trust) |
| `SAMPLE_RATE_HZ` | 100 | IMU sampling frequency |

### Gyro Calibration

In `src/imu/mpu6050.h`:
- `GYRO_CONST_ERROR_MEAS` - Set to negative of gyro reading when stationary

## License

See [LICENSE](LICENSE) for details.
