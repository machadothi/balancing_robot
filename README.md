# Balancing Robot

Self-balancing two-wheeled robot firmware for STM32 and FreeRTOS: MPU-6050
sensor fusion (complementary or Kalman filter), a discrete PID balance loop,
and an AT command console for live tuning. One source tree builds for two
boards.

![Balancing Robot](img/balancing_robot.png)

## Boards

| Board | MCU | Status | Wiring |
|-------|-----|--------|--------|
| Blue Pill + TB6612FNG + MPU-6050 module | STM32F103C8T6 | Balancing | [pin-connections-f103](docs/hardware/pin-connections-f103.md) |
| Hiwonder ROS Robot Control Board | STM32F407VET6 | Full firmware builds; hardware bring-up in progress | [pin-connections-f407](docs/hardware/pin-connections-f407.md) |

## Quick start

```bash
git clone --recurse-submodules https://github.com/machadothi/balancing-robot.git
cd balancing-robot

# Build libopencm3 for both MCU families
cd lib/libopencm3 && make TARGETS="stm32/f1 stm32/f4" -j$(nproc) && cd ../..

# Configure, build and flash (preset f103 or f407)
cmake --preset f103
cmake --build --preset f103
cmake --build --preset f103 --target flash
```

Then open the console at 921600 baud and try `AT+STATUS?`. Details, options and
the serial bootloader are in
[docs/02 — Build and Configuration](docs/02-build-and-configuration.md).

## Documentation

The [`docs/`](docs/README.md) folder is a book about this robot, written for
embedded engineers:

1. [System Overview](docs/01-system-overview.md)
2. [Build and Configuration](docs/02-build-and-configuration.md)
3. [Boot and RTOS](docs/03-boot-and-rtos.md)
4. [Drivers and Board Layer](docs/04-drivers-and-board-layer.md)
5. [Sensing and the IMU](docs/05-sensing-and-imu.md)
6. [Control Theory](docs/06-control-theory.md)
7. [Sensor Fusion](docs/07-sensor-fusion.md)
8. [PID Implementation](docs/08-pid-implementation.md)
9. [Tuning and Experiments](docs/09-tuning-and-experiments.md)
10. [AT Commands](docs/10-at-commands.md)

## Repository layout

| Path | Contents |
|------|----------|
| `src/` | Firmware: `app`, `board`, `drivers`, `imu`, `filter`, `robot`, `motor`, `cmd`, `fault`, `rtos` |
| `cmake/` | Toolchain file and per-board settings (`cmake/boards/`) |
| `CMakePresets.json` | One configure/build preset per board |
| `lib/` | libopencm3 and FreeRTOS-Kernel submodules |
| `docs/` | Documentation book and hardware references |
| `test/` | Filter analysis script |
| `scripts/` | Setup script |
| `vendor/` | Original firmware image of the Hiwonder board |

## License

See [LICENSE](LICENSE) for details.
