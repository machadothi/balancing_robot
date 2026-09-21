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
- [ ] F407 Bluetooth console: measure the Bluetooth header's supply voltage,
      set the HC-05 to `BT_BAUDRATE`, then run
      `pytest --port /dev/rfcomm0 --bluetooth`
      ([10](docs/10-at-commands.md#bluetooth-console-f407-board)).
- [ ] Log a long balancing run over USB and confirm `test_no_lost_records`
      holds under real load on both boards.
- [ ] Verify the `flash-serial` DTR/RTS boot sequence (`SERIAL_BOOT_SEQUENCE`)
      and update [02](docs/02-build-and-configuration.md) with the working value.
      Blue Pill only: the F407 Type-C port is USART3 on PD8/PD9, which the ROM
      bootloader does not serve, so the F407 is flashed over SWD.
- [x] Back up the F407 vendor firmware before the first flash
      (`~/git/hiwonder-vendor-fw/vendor_fw.bin`, RDP level 0, read twice and
      identical; restore with `st-flash write vendor_fw.bin 0x08000000`).

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
- [ ] Arbitrate the USB and Bluetooth consoles, and stop the robot when the
      Bluetooth link drops (HC-05 STATE pin).
- [ ] DMA transmission for the USB console if the per-byte TX interrupt load
      matters on the Blue Pill.

## 6. Instrumentation and tests

- [ ] Add per-wheel PWM and encoder counts to the telemetry record
      ([09](docs/09-tuning-and-experiments.md#what-to-observe)).
- [ ] Host script that captures telemetry to CSV and plots tilt and PID terms.
- [ ] [test/filter_comparison.py](test/filter_comparison.py): reuse `at_console.py` (port and
      baud options, sends `AT+STREAM=1` itself) and update its docstring to the
      current `acc_deg | kalman | comp` format.
- [x] Host unit tests for the pure C modules (`ctest --preset host`, test/host/).
- [x] CI: host tests, doc links and `scripts/build_matrix.sh` (.github/workflows/ci.yml).
- [ ] Watch the first CI run on GitHub (libopencm3 cache key, toolchain version).
- [ ] Host tests for `telemetry_submit()` drops/sequence and `uart_write()` ring
      buffer wrap-around (need a queue shim).

## 7. Documentation

- [ ] Preview all Mermaid diagrams on GitHub (not rendered locally yet).
- [x] Run `scripts/check_docs.py` in CI (`--fix` re-anchors `file#Lnn` links
      after code moves; review the result, it matches symbols heuristically).
- [ ] Install `doxygen graphviz` and check the `docs` target output.
- [ ] Add a photo or schematic of the F407 robot wiring once it is built.

## 8. Board features from the vendor firmware (F407)

The vendor firmware (decompiled in `~/git/hiwonder-vendor-fw/decompiled/vendor_fw.c`)
drives more of the board than we do. Each item fits as one module: an
`APP_MODULE()` descriptor plus a `robot_feature()` flag ([01](docs/01-system-overview.md#adding-a-module)).
Pins marked *(?)* are inferred from GPIO setup only, so confirm them in the
decompiled code before use.

- [ ] **Battery monitor** (`battery_check_timer`, prints `BAT:%dmv`): ADC1 on
      PB0 (IN8). Find the divider ratio in the vendor code, then add `AT+BAT?`, a
      telemetry field and a low-battery cut-off that disables the motors (a
      sagging supply makes the motor gain drift during tuning).
- [ ] **Buzzer** (`buzzer_timer`, `buzzer1_ctrl_quque`): PA8, software PWM from
      the TIM13 interrupt at 200 Hz. Beep on enable, fall and low battery.
- [ ] **OLED display** (`oled_task`, `gui_task`, LVGL): SPI2, SCK PB13 and MOSI
      PC3; CS/DC/RST among PD11–PD14, PC8 and PC9 *(?)*; controller chip
      unknown (SSD1306 likely). A small text screen is enough, no LVGL: tilt,
      battery, state, gains.
- [ ] **Buttons** (`button_timer`): inputs on PE0, PE1 and PD3 *(?)*. Enable or
      disable balancing without a console; a gain preset selector.
- [ ] **Status LEDs** (`led_timer`, `led1_ctrl_quque`): PE10 is ours; PE7 and
      PE8 are more outputs *(?)*. Blink patterns for disabled, balancing, fault
      and low battery.
- [ ] **IMU data-ready interrupt** (`mpu6050_data_ready`): PB12, rising edge.
      Sample on data-ready instead of the timer: less jitter, and the delay
      between measurement and control becomes fixed.
- [ ] **RC receiver** (`sbus_rx_task`): UART5 RX on PD2, 100000 baud 8E2, receive
      only (vendor setting; SBUS is also inverted). Drive and turn from a radio
      remote through `target_velocity` and `turn_rate`.
- [ ] Lower priority:
      - bus servos (`serial_servo_rx_complete`, USART3 on PD8/PD9 at 1 Mbaud)
      - USART6 (PC6/PC7, 115200): purpose unknown, maybe the Raspberry Pi header
      - USB-host gamepad (`USBH_Queue`, PA11/PA12)
      - the vendor's PC packet protocol (`packet_rx_task`/`packet_tx_task`),
        for compatibility with their ROS tools
