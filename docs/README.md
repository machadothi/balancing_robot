# Balancing Robot — Engineering Notes

A walkthrough of the firmware for engineers: how the hardware, RTOS, drivers
and control loop fit together, and the control theory behind balancing,
always tied back to the code.

## Chapters

| # | Chapter | What it covers |
|---|---------|----------------|
| 01 | [System Overview](01-system-overview.md) | Hardware of both boards, firmware layers, one control cycle end to end |
| 02 | [Build and Configuration](02-build-and-configuration.md) | Toolchain, CMake presets, build options, flashing, API docs |
| 03 | [Boot and RTOS](03-boot-and-rtos.md) | Start-up sequence, tasks and priorities, queues, locks, interrupt priorities, timing |
| 04 | [Drivers and Board Layer](04-drivers-and-board-layer.md) | Board abstraction, F1 vs F4 differences, I2C with DMA, UART, PWM and encoders, porting |
| 05 | [Sensing and the IMU](05-sensing-and-imu.md) | MPU-6050 configuration, scaling, axes, calibration |
| 06 | [Control Theory](06-control-theory.md) | Inverted pendulum model, stability, delay, state space, LQR, cascade control |
| 07 | [Sensor Fusion](07-sensor-fusion.md) | Complementary and Kalman filters derived, tuning, bench lab |
| 08 | [PID Implementation](08-pid-implementation.md) | The discrete PID as coded: anti-windup, saturation, deadband, safety |
| 09 | [Tuning and Experiments](09-tuning-and-experiments.md) | Bring-up checklist, tuning procedure, symptom → cause |
| 10 | [AT Commands](10-at-commands.md) | Console command reference |

Hardware references: [Blue Pill wiring](hardware/pin-connections-f103.md),
[Hiwonder F407 board pinout](hardware/pin-connections-f407.md).

## Reading paths

| Goal | Path |
|------|------|
| Understand the firmware architecture | 01 → 02 → 03 → 04 |
| Learn the control engineering | 01 → 06 → 07 → 08 → 09 |
| Port to a new board | 01 → 02 → 04 (porting checklist) → hardware pages |
| Just build, flash and tune a robot | 02 → 09 → 10 |

## Conventions

- **Code links** point at line numbers in the current tree. If a link has
  drifted after an edit, search for the function name it names.
- **Units.** The firmware uses degrees and °/s; the theory chapters use radians
  where equations require it. PWM commands are in counts, ±255 full scale.
- **Tilt sign.** 0° is upright; positive means leaning forward, the direction
  the wheels must drive to catch the robot.
- **Diagrams** are [Mermaid](https://mermaid.js.org/) blocks and equations use
  GitHub math blocks; both render on GitHub and in most Markdown previewers.
- **"Limitations / next steps"** close each chapter: known gaps are stated,
  not hidden.

## API reference

Doxygen generates a browsable API reference from the source comments:

```bash
sudo apt install doxygen graphviz
cmake --preset f103
cmake --build --preset f103 --target docs
xdg-open build-f103/docs/html/index.html
```
