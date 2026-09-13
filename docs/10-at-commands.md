# 10 — AT Command Reference

AT commands are accepted on every console port:

| Console | Blue Pill | F407 board | Carries |
|---------|-----------|------------|---------|
| USB | USART2 via a USB-serial adapter | Type-C USB-serial port (USART1) | AT commands, telemetry, startup banner, echo |
| Bluetooth | — | Bluetooth header (USART2) | AT commands only, no echo |

Commands are case-insensitive, run one at a time in the UART RX task under the
robot state lock, and each reply goes back to the console the command came
from (see [03 — Boot and RTOS](03-boot-and-rtos.md)).

## Where in the code

| What | Where |
|------|-------|
| Line parsing and dispatch | [`at_cmd_process()`](../src/cmd/at_cmd.c#L182) |
| Queries (`AT+X?`) | [`at_handle_query()`](../src/cmd/at_cmd.c#L351) |
| Set commands (`AT+X=v`), range checks | `at_handle_set()` in [at_cmd.c](../src/cmd/at_cmd.c) |
| Execute commands (`AT+X`) | [`at_handle_execute()`](../src/cmd/at_cmd.c#L549) |
| What set/execute commands do | [`at_set_handler()`](../src/robot/robot.c#L247), [`at_exec_handler()`](../src/robot/robot.c#L298) |
| Build flags | `AT_CMD_HELP_ENABLED`, `AT_CMD_ALL_QUERY`, `AT_CMD_PID_TOGGLE`, `UART_ECHO_ENABLED` ([02](02-build-and-configuration.md#build-options)) |

## Syntax

| Format | Meaning | Example |
|--------|---------|---------|
| `AT` | Connection test | `AT` → `OK` |
| `AT+CMD?` | Query | `AT+ANGLE?` → `+ANGLE:1.23` |
| `AT+CMD=value` | Set | `AT+KP=20` → `OK` |
| `AT+CMD` | Execute | `AT+ENABLE` → `OK` |

Lines end with CR or LF, and `> ` is printed when the console is ready for the
next command. The USB console echoes each command line when it is received
(`UART_ECHO_ENABLED`), not character by character: enable local echo in your
terminal to see what you type.

### Responses and error codes

| Response | Meaning |
|----------|---------|
| `OK` | Success |
| `+CMD:value` | Query result (followed by the prompt) |
| `ERROR:1` | General error (command known but failed, e.g. `AT+SAVE`) |
| `ERROR:2` | Unknown command |
| `ERROR:3` | Invalid parameter: missing, not a number, or not finite (`AT+KP=abc`, `AT+KP=nan`) |
| `ERROR:4` | Value out of range |
| `ERROR:5` | Not ready (robot task has not registered its handlers yet) |

The numbers are the `AT_Result_t` values in [at_cmd.h](../src/cmd/at_cmd.h).

## Queries

| Command | Response | Notes |
|---------|----------|-------|
| `AT+VERSION?` | `+VERSION:1.0.0` | |
| `AT+STATUS?` | `+STATUS:ENABLED,BALANCED` | Motors `ENABLED`/`DISABLED`; `BALANCED` when \|tilt\| < 5° |
| `AT+ACC_X?` `AT+ACC_Y?` `AT+ACC_Z?` | `+ACC_X:-0.998` | g, 3 decimals |
| `AT+GYRO_X?` `AT+GYRO_Y?` `AT+GYRO_Z?` | `+GYRO_X:0.125` | °/s, 3 decimals; X includes the calibration offset |
| `AT+ANGLE?` | `+ANGLE:1.23` | Filtered tilt, 0 = upright, positive = leaning forward |
| `AT+KP?` `AT+KI?` `AT+KD?` | `+KP:25.0000` | Current PID gains |
| `AT+TURN?` | `+TURN:0.00` | |
| `AT+SPEED?` | `+SPEED:30.0,30.0` | Last values set with `AT+SPEED=` |
| `AT+TARGET?` | `+TARGET:0.00` | Stored only, see limitations |
| `AT+VELOCITY?` | `+VELOCITY:0.00` | Always 0: velocity is not estimated yet |
| `AT+ALL?` | `+ALL:ax,ay,az,gx,gy,gz,angle` | Only with `AT_CMD_ALL_QUERY=ON` (default OFF) |

Queries copy the robot state under the lock and format the copy, so values in
one response are consistent with each other.

## Set commands

| Command | Range | Effect |
|---------|-------|--------|
| `AT+KP=n` `AT+KI=n` `AT+KD=n` | ≥ 0 | PID gains, effective on the next sample |
| `AT+TURN=n` | −100 … 100 | Added to the left wheel and subtracted from the right |
| `AT+SPEED=l,r` | −100 … 100 each | Drives the wheels directly (see [Direct wheel control](#direct-wheel-control)) |
| `AT+VELOCITY=n` / `AT+TARGET=n` | −100 … 100 | Stored in `target_velocity`, not used by the control law |
| `AT+STREAM=0\|1` | 0 or 1 | Telemetry record every sample, on the USB console (from either console) |

## Execute commands

| Command | Effect |
|---------|--------|
| `AT+ENABLE` | Reset the PID state, leave motor standby, start balancing (if the PID is on) |
| `AT+DISABLE` | Stop balancing and put the motor driver in standby |
| `AT+STOP` | Like `DISABLE`, and also zero wheel speeds, `TURN` and `TARGET` |
| `AT+PID` | Toggle the balance PID (`AT_CMD_PID_TOGGLE`, default ON) |
| `AT+PIDON` / `AT+PIDOFF` | Enable / disable the PID; `PIDOFF` also zeroes the motors |
| `AT+DEFAULT` | Restore the default gains (Kp 25, Ki 0.5, Kd 0.8) and zero `TURN`/`TARGET` |
| `AT+RESET` | Reply `OK`, wait 100 ms, then reset the MCU |
| `AT+SAVE` / `AT+LOAD` | `ERROR:1`: parameter storage is not implemented |
| `AT+HELP` | Command summary, only with `AT_CMD_HELP_ENABLED=ON` (default OFF) |

The robot starts with motors in standby: nothing moves until `AT+ENABLE`.
Balancing also stops by itself when |tilt| exceeds 45° or when the IMU delivers
no sample for 50 ms; send `AT+ENABLE` again once the cause is gone.

## Examples

### Check the sensors

```
AT+STATUS?
+STATUS:DISABLED,UNBALANCED
AT+ANGLE?
+ANGLE:0.84
AT+GYRO_X?
+GYRO_X:0.031
```

### Tuning session

```
AT+KD=0
OK
AT+KI=0
OK
AT+KP=15
OK
AT+ENABLE
OK
AT+KD=0.6
OK
AT+KP?
+KP:15.0000
```

Gains are lost on reset: write the final values into `ROBOT_DEFAULT_KP/KI/KD`
in [robot.c](../src/robot/robot.c#L38). The procedure is in
[09 — Tuning and Experiments](09-tuning-and-experiments.md).

### Direct wheel control

The balance loop overwrites the motor commands every sample while it is
active, so turn the PID off first. `AT+ENABLE` is still needed to leave standby:

```
AT+PIDOFF
OK
AT+ENABLE
OK
AT+SPEED=30,30
OK
AT+SPEED=-30,30
OK
AT+STOP
OK
```

### Stream filter data for `test/filter_comparison.py`

```
AT+STREAM=1
OK
seq: 1041 | t: 10410 | acc_deg: 90.84 | kalman: 90.61 | comp: 90.58 | tilt: 0.58 | p: -14.50 | i: -0.03 | d: 2.10 | out: -12.43 | drops: 0
seq: 1042 | t: 10420 | acc_deg: 90.77 | kalman: 90.66 | comp: 90.59 | tilt: 0.59 | p: -14.75 | i: -0.03 | d: -2.50 | out: -17.28 | drops: 0
```

| Field | Meaning |
|-------|---------|
| `seq` | Record number; a gap means a lost record |
| `t` | Milliseconds since boot when the sample was processed |
| `acc_deg`, `kalman`, `comp` | Accelerometer angle and both filter outputs, raw (upright ≈ 90°, see [05](05-sensing-and-imu.md)) |
| `tilt` | Angle used by the controller (0 = upright) |
| `p`, `i`, `d`, `out` | PID terms and clamped output, in PWM counts; 0 while not balancing |
| `drops` | Records the firmware dropped because the logger fell behind, since boot |

Telemetry always goes to the USB console, whichever console enabled it. Whole
lines are written, so records never corrupt command replies, but they do appear
between them. [test/filter_comparison.py](../test/filter_comparison.py) reads
the `acc_deg`, `kalman` and `comp` fields.

## Bluetooth console (F407 board)

An HC-05 or HC-06 module (for example on a ZS-040 breakout) on the board's
Bluetooth header gives a wireless AT console alongside USB.

1. **Supply.** The ZS-040 needs 3.6–6 V on VCC. Measure the header's supply pin
   first; if it provides only 3.3 V, power the module from a 5 V pin instead.
2. **Wiring.** Module TXD → PD6, module RXD → PD5, GND → GND. The logic is 3.3 V,
   no level shifting needed.
3. **Baud rate.** Set the module to `BT_BAUDRATE` (115200 by default):
   - HC-05: hold its button while powering up (AT mode, 38400 baud, CR+LF line
     endings) and send `AT+UART=115200,0,0`.
   - HC-06: send `AT+BAUD8` at 9600 baud with no line ending.
4. **Connect.** Pair (PIN usually 1234). On Linux:
   `sudo rfcomm bind 0 <MAC>`, then open `/dev/rfcomm0`; its baud setting is
   ignored. Classic Bluetooth serial (SPP) works with Android terminal apps but
   not with iPhones.

The Bluetooth console does not echo and never receives telemetry. Run the
hardware tests over it with `pytest --port /dev/rfcomm0 --bluetooth`.

## Limitations

- No persistence: `AT+SAVE`/`AT+LOAD` return `ERROR:1`.
- `VELOCITY`/`TARGET` have no effect: there is no outer velocity loop yet
  ([06 — Control Theory](06-control-theory.md#6-cascade-control-the-next-step)).
- `AT+SPEED?` reports the last commanded values, not measured wheel speeds.
- The two consoles are equal: the last command wins, and losing the Bluetooth
  link does not stop the robot. Keep a USB `AT+STOP` ready while driving over
  Bluetooth.
