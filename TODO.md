# TODO

Recommended next steps, most important first. Background for each item is in
the linked [docs](docs/README.md) chapter.

## 1. Next on the F407 robot

- [ ] **Battery operation** (in progress: wiring). Then:
      - battery monitor and low-voltage cut-off (section 8);
      - re-measure the dead zone on battery voltage (`pid_tune.py deadband --write`);
      - re-check KP/KD with one sweep each ([09](docs/09-tuning-and-experiments.md#pid-tuning-procedure)).
- [ ] **Driving:** the Android app ([android/](android/README.md)) has the
      jog; try it with a small max speed. The firmware has a 1 s dead-man on
      `AT+VELOCITY`/`AT+TURN`. Then: ramp the speed target in the firmware so a
      stick step does not tip the robot.
- [x] Firmware with `AT+LIVE?` and the drive dead-man flashed and tested (USB
      and Bluetooth, ~8 `AT+LIVE?` replies/s); module renamed "balancing robot".
- [ ] Try the app on the phone: connect, Live tab, Settings read/write, then the
      jog with a small max speed.
- [ ] **AtomS3R-CAM bridge** ([atom/](atom/README.md), `BT_MODULE=atom`):
      firmware runs (BLE "balancing robot", GC0308 camera found, local commands
      tested over BLE from the PC); app has BLE scan, camera view and Wi-Fi setup.
      Next: measure the header's VCC, wire it, flash the robot with
      `BT_MODULE=atom`, connect from the app, set the Wi-Fi, check the video
      (orientation, frame rate) and the robot's 5 V under the extra load.
- [ ] Show `ARMED` in `AT+STATUS?`: a button press that did not register is
      invisible today.
- [ ] Parameter storage in flash for `AT+SAVE` / `AT+LOAD`, so tuning survives a
      power cycle without a rebuild.

## 2. Verify on hardware

- [x] F407 bring-up: LED, console on USB-C (USART3), QMI8658 IMU, motors and
      encoders, Bluetooth (HC-06 at 9600), button, balancing in place
      ([09 case study](docs/09-tuning-and-experiments.md#case-study-the-f407-robot)).
- [x] F407: own bootloader in sector 0, updates over USB-C with `flash-usb`
      (tested: normal update 8.7 s, interrupted update recovered).
- [x] Back up the F407 vendor firmware before the first flash
      (`~/git/hiwonder-vendor-fw/vendor_fw.bin`, RDP level 0, read twice and
      identical; restore with `st-flash write vendor_fw.bin 0x08000000`).
- [ ] **Blue Pill regression**: flash `f103`, confirm it still balances. Since
      its tuning: control-law sign fix with its tilt mapping negated (should be
      neutral), 1 kHz tick, 42 Hz sensor low-pass, gyro bias measured at power-on,
      continuous dead-zone compensation.
- [ ] Run the hardware test suite on both boards: `pytest` (robot still), then
      `--motors` (lifted), then `--motors --interactive`
      ([test/README.md](test/README.md)), and over Bluetooth with
      `--port /dev/rfcomm0 --bluetooth`.
- [ ] Log a long balancing run over USB and confirm `test_no_lost_records`
      holds under real load.
- [ ] F407: try `flash_usb.py --power-cycle` (recovery when the firmware hangs).
- [ ] F407: firmware updates over Bluetooth (9600 baud, about 1 minute).
- [ ] Blue Pill: verify the `flash-serial` DTR/RTS boot sequence
      (`SERIAL_BOOT_SEQUENCE`) and update [02](docs/02-build-and-configuration.md).

## 3. Control improvements

- [x] D term from the gyro rate (`AT+DGYRO`, F407 default).
- [x] Continuous dead-zone compensation, per wheel, measured with `pid_tune.py deadband`.
- [x] Outer speed loop from the encoders (`AT+VLOOP`, `AT+VKP/VKI`), tuned on
      the F407 robot: it stays in place.
- [x] Complementary filter weight per board (`AT+ALPHA`): the wheels'
      acceleration fed back through the accelerometer.
- [ ] Compensate the accelerometer for the wheel acceleration measured by the
      encoders, then α could come down again ([07](docs/07-sensor-fusion.md#what-wheel-acceleration-does)).
- [ ] Per-wheel speed trim: the M1 motor runs ~4-5 % slower at the same command
      (a slow turn when driving straight).
- [ ] Conditional integration while the output is saturated (speed loop).
- [ ] Optional: discrete LQR on [θ, θ̇, x, ẋ] as a comparison to the cascade.
- [ ] Measure the robot's effective pendulum length; replace the illustrative
      values in [06](docs/06-control-theory.md).

## 4. Sensing

- [ ] Retune the Kalman filter from measured variances (R from `acc_deg` at rest,
      Q from gyro noise × T²) and re-run the filter comparison
      ([07 §3](docs/07-sensor-fusion.md#3-the-kalman-filter-as-implemented)).
- [x] Measure gyro bias at start-up while the robot is still, instead of the
      fixed `GYRO_CALIBRATION_OFFSET`. Done: 1 s average at power-on, `AT+GYROBIAS?`.
- [ ] Two-state Kalman filter (angle + gyro bias)
      ([07 §4](docs/07-sensor-fusion.md#4-the-next-step-estimating-the-gyro-bias)).
- [ ] Re-initialise the IMU after repeated read failures instead of staying
      stopped.
- [ ] QMI8658: gravity reads 0.91 g on Z; an accelerometer offset/scale
      calibration (flat and flipped) would correct it.

## 5. Firmware robustness

- [ ] Fix or remove the multi-byte path of polled `i2c_read()` (index incremented
      twice per byte).
- [ ] Blue Pill: the TB6612 PWM pins are configured open-drain
      ([motor.c](src/motor/motor.c)). Confirm the board has pull-ups, otherwise
      switch to push-pull.
- [ ] Measure task stack usage (`INCLUDE_uxTaskGetStackHighWaterMark`) and size
      stacks from data.
- [ ] Decide whether the watchdog should also supervise the UART tasks.
- [ ] Arbitrate the USB and Bluetooth consoles, and stop the robot when the
      Bluetooth link drops (module STATE pin).
- [ ] DMA transmission for the USB console if the per-byte TX interrupt load
      matters on the Blue Pill.

## 6. Instrumentation and tests

- [x] Tuning tool: [test/pid_tune.py](test/pid_tune.py) captures telemetry to
      CSV, analyses and plots it, sweeps gains, measures the dead zone and noise.
- [x] Speed (`v`) and effective setpoint (`spe`) in the telemetry record.
- [ ] Add per-wheel PWM and encoder counts to the telemetry record
      ([09](docs/09-tuning-and-experiments.md#what-to-observe)).
- [ ] `pid_tune.py`: a `wait` helper (start when balancing begins) instead of
      the inline scripts used during tuning; skip the setpoint advice on falls.
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
- [x] **Enable button**: PE0 (active low) arms balancing, which starts when the
      robot is lifted upright; a second press stops ([button.c](src/ui/button.c)).
- [ ] Second button PE1 (PD3 is a third input *(?)*): a gain preset selector, or
      calibrate the gyro bias on demand.
- [ ] Show the armed state (fast LED blink or buzzer beep) once those modules exist.
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
      - bus servos (`serial_servo_rx_complete`): USART6 on PC6/PC7, 115200,
        half-duplex with PE7/PE8 as direction enables (pin doc)
      - USB-host gamepad (`USBH_Queue`, PA11/PA12)
      - the vendor's PC packet protocol (`packet_rx_task`/`packet_tx_task`),
        for compatibility with their ROS tools
