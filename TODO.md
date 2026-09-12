# TODO

Recommended next steps, most important first. Background for each item is in
the linked [docs](docs/README.md) chapter.

## 1. Commit and verify on hardware

- [ ] Commit the pending work in three commits: F407 port + CMake presets;
      control-path fixes + docs book; safety fixes (IMU stall, fault motor stop,
      watchdog, I2C timeouts, AT input validation).
- [ ] **Blue Pill regression**: flash `f103`, confirm it still balances with the
      new 1 kHz tick, 42 Hz sensor low-pass filter and task priorities. The extra
      ~5 ms of sensor delay may need slightly more `KD`
      ([06 §4](docs/06-control-theory.md#4-going-digital-sampling-and-delay)).
- [ ] Run the hardware test suite on both boards: `pytest` (robot still), then
      `--motors` (lifted), then `--motors --interactive`
      ([test/README.md](test/README.md)). It covers the safety checks: IMU-loss
      and tilt cut-offs, no watchdog resets, `ERROR:3` for `nan`/garbage
      parameters, `AT+PIDON`/`AT+PIDOFF`/`AT+SAVE` replies.
- [ ] **F407 bring-up** in order: LED blink → console on Type-C → `AT+ANGLE?` →
      motor direction per port → balancing
      ([09](docs/09-tuning-and-experiments.md#bring-up-checklist)).
- [ ] Verify the `flash-serial` DTR/RTS boot sequence (`SERIAL_BOOT_SEQUENCE`)
      and update [02](docs/02-build-and-configuration.md) with the working value.
- [ ] Back up the F407 vendor firmware before the first flash
      (`st-flash read vendor_backup.bin 0x8000000 0x80000`).

## 2. Hardware questions to settle

- [ ] Blue Pill: the TB6612 PWM pins are configured open-drain
      ([motor.c](src/motor/motor.c)). Confirm the board has pull-ups, otherwise
      switch to push-pull.
- [ ] Measure the robot's effective pendulum length and motor deadband; replace
      the illustrative values in [06](docs/06-control-theory.md) and
      `MOTOR_DEADBAND`.

## 3. Control improvements

Ordered by expected payoff ([08](docs/08-pid-implementation.md#limitations-and-next-steps)):

- [ ] Use the gyro rate as the D input instead of differencing the angle.
- [ ] Continuous deadband compensation (offset mapping instead of 1…19 → 20).
- [ ] Conditional integration while the output is saturated.
- [ ] Velocity estimate from the encoders, then an outer velocity PI loop so
      `AT+VELOCITY` works and the robot stops drifting
      ([06 §6](docs/06-control-theory.md#6-cascade-control-the-next-step)).
- [ ] Optional: discrete LQR on [θ, θ̇, x, ẋ] as a comparison to the cascade.

## 4. Sensing

- [ ] Retune the Kalman filter from measured variances (R from `acc_deg` at rest,
      Q from gyro noise × T²) and re-run the filter comparison
      ([07 §3](docs/07-sensor-fusion.md#3-the-kalman-filter-as-implemented)).
- [ ] Measure gyro bias at start-up while the robot is still, instead of the
      fixed `GYRO_CALIBRATION_OFFSET`.
- [ ] Two-state Kalman filter (angle + gyro bias)
      ([07 §4](docs/07-sensor-fusion.md#4-the-next-step-estimating-the-gyro-bias)).
- [ ] Re-initialise the IMU after repeated read failures instead of staying
      stopped.

## 5. Firmware robustness

- [ ] Fix or remove the multi-byte path of polled `i2c_read()` (index incremented
      twice per byte).
- [ ] Parameter storage in flash for `AT+SAVE` / `AT+LOAD` (gains, calibration).
- [ ] Measure task stack usage (`INCLUDE_uxTaskGetStackHighWaterMark`) and size
      stacks from data.
- [ ] Decide whether the watchdog should also supervise the UART tasks.
- [ ] Replace `uart_printf`'s binary semaphore with a mutex (priority
      inheritance).

## 6. Instrumentation and tests

- [ ] Extend `AT+STREAM` with PID terms and PWM, keeping lines short
      ([09](docs/09-tuning-and-experiments.md#what-to-observe)).
- [ ] [test/filter_comparison.py](test/filter_comparison.py): reuse `at_console.py` (port and
      baud options, sends `AT+STREAM=1` itself) and update its docstring to the
      current `acc_deg | kalman | comp` format.
- [ ] Host unit tests for the pure C modules (filters, PID, `at_format_fixed()`,
      AT parsing), complementing the hardware tests in `test/`.
- [ ] CI: build both presets (plus `-DATTITUDE_FILTER=kalman` and all AT flags ON)
      with warnings as errors.

## 7. Documentation

- [ ] Preview all Mermaid diagrams on GitHub (not rendered locally yet).
- [ ] Move the docs link/anchor checker into `scripts/` and run it in CI, since
      `file#Lnn` links drift with every code change.
- [ ] Install `doxygen graphviz` and check the `docs` target output.
- [ ] Add a photo or schematic of the F407 robot wiring once it is built.
