# Hardware Tests

A pytest suite that checks a flashed robot through its AT command console:
protocol and error handling, IMU sanity, control loop rate, filter behaviour,
watchdog stability, and (opt-in) motor and safety cut-off tests. The robot's
behaviour is described in [docs/10-at-commands.md](../docs/10-at-commands.md).

`filter_comparison.py` is the separate filter comparison script used in
[docs/07-sensor-fusion.md](../docs/07-sensor-fusion.md#5-lab-compare-the-filters-on-your-robot).

## Setup

```bash
cd test
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

On Debian/Ubuntu, `python3 -m venv` needs the `python3-venv` package
(`sudo apt install python3-venv`).

Close any serial terminal first: only one program can own the port.

## Running

```bash
# Safe tests: nothing moves. Hold the robot still for the sensor and stream tests.
pytest --port /dev/ttyUSB0

# Also drive the wheels: lift the robot so they spin freely
pytest --port /dev/ttyUSB0 --motors

# Everything, including tests that ask you to tilt the robot, unplug the IMU or let it balance
pytest --port /dev/ttyUSB0 --motors --interactive

# Over the F407 board's Bluetooth console (skips telemetry and banner tests)
pytest --port /dev/rfcomm0 --bluetooth

# One file or one test
pytest test_console.py
pytest -k fixed_point
```

Skipped tests are listed with their reason (for example "pass --motors") in the
summary.

| Option | Default | Meaning |
|--------|---------|---------|
| `--port` | `$ROBOT_PORT` or `/dev/ttyUSB0` | Console serial port |
| `--baud` | `$ROBOT_BAUD` or `921600` | Must match the firmware's `UART_BAUDRATE` |
| `--motors` | off | Run tests that drive the wheels |
| `--interactive` | off | Run tests that need an operator |
| `--bluetooth` | off | `--port` is the Bluetooth console (firmware built with `CONSOLE_BT=ON`): skip tests that need USB telemetry or the banner |
| `--assert-dtr-rts` | off | Assert DTR/RTS when opening the port (see below) |

## What each file checks

| File | Robot must be | Checks |
|------|---------------|--------|
| [test_console.py](test_console.py) | Anywhere | `OK`/data/error replies, every error code, rejection of `nan`/garbage parameters, gain round-trip and `AT+DEFAULT`, fixed-point formatting |
| [test_sensors.py](test_sensors.py) | Still | Gravity magnitude ≈ 1 g, gyro bias at rest, live (not frozen) data, filtered angle vs accelerometer |
| [test_stream.py](test_stream.py) | Still | USB only. No lost records (`seq` gaps, firmware `drops`), record rate = loop rate (100 Hz), 10 ms period jitter, finite values, filters agree with the accelerometer, complementary filter smoother than raw |
| [test_safety.py](test_safety.py) | Still | No watchdog reset idle or under load, ENABLE/STOP; with `--interactive`: tilt cut-off, IMU-loss cut-off; with both flags: balances for 3 s |
| [test_motors.py](test_motors.py) | Lifted | Wheel direction patterns, `AT+SPEED?` reporting, `AT+STOP` zeroes speeds |

After every test the suite sends `AT+STREAM=0`, `AT+STOP`, `AT+PIDON` and
`AT+DEFAULT`, so a failing test never leaves the motors running or modified
gains behind.

## Keep in sync with the firmware

These constants mirror firmware settings; update them if you change those:

| Test constant | Firmware setting |
|---------------|------------------|
| `DEFAULT_GAINS` in `test_console.py` | `ROBOT_DEFAULT_KP/KI/KD` in `src/robot/robot.c` |
| `SAMPLE_PERIOD_MS` in `test_stream.py` | `IMU_SAMPLE_RATE_MS` |
| `--baud` | `UART_BAUDRATE` |

Tests that use `AT+PIDON`/`AT+PIDOFF` need `AT_CMD_PID_TOGGLE=ON` (the default).

## Troubleshooting

| Symptom | Cause |
|---------|-------|
| `cannot open /dev/ttyUSB0` | Wrong port, or a terminal still has it open; on Linux add yourself to the `dialout` group |
| `no AT console` / timeouts | Wrong `--baud`, board not running, or TX/RX swapped (Blue Pill console is USART2 on PA2/PA3) |
| F407 board resets or stops answering when the port opens | Its USB-serial auto-bootloader circuit: keep DTR/RTS released (the default) |
| All sensor tests fail with `0,0,0` | IMU not initialised: check wiring, the console log, or `AT+ACC_X?` by hand |
| `motors disabled themselves` | No IMU samples for 50 ms triggered the stall cut-off |
| Low `test_loop_rate` | Control loop overrun, or telemetry dropped at a low baud rate |

## Host unit tests (no hardware)

[host/](host/) builds the hardware-independent modules with the native compiler,
AddressSanitizer and UBSan, against small stand-ins for FreeRTOS and libopencm3
([host/shim/](host/shim/)):

```sh
cmake --preset host && cmake --build --preset host && ctest --preset host
```

| Test | Covers |
|------|--------|
| [test_control.c](host/test_control.c) | PID worked example from [docs/08](../docs/08-pid-implementation.md), output and integral clamps, reset; mixer saturation and deadband |
| [test_filters.c](host/test_filters.c) | Every `AttitudeFilter_t`: seeding, convergence, tracking a rotation, rejecting an accelerometer spike |
| [test_fmt.c](host/test_fmt.c) | `fmt_fixed()` rounding and sign |
| [test_at.c](host/test_at.c) | AT syntax, error codes, gains, motor states and `AT+HELP`: the protocol cases of `test_console.py` |

A new pure C module gets a `host_test()` line in [host/CMakeLists.txt](host/CMakeLists.txt).

`scripts/build_matrix.sh` builds both boards in feature combinations and checks
that invalid ones are rejected. CI ([.github/workflows/ci.yml](../.github/workflows/ci.yml))
runs the host tests, the matrix and `scripts/check_docs.py`.
