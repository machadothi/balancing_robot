# 10 — AT Command Reference

The console accepts AT commands over the board's console UART: USART2 on the
Blue Pill, the Type-C USB-serial port (USART1) on the F407 board. Commands are
case-insensitive and are executed by the UART RX task under the robot state
lock (see [03 — Boot and RTOS](03-boot-and-rtos.md)).

## Where in the code

| What | Where |
|------|-------|
| Line parsing and dispatch | [`at_cmd_process()`](../src/cmd/at_cmd.c#L199) |
| Queries (`AT+X?`) | [`at_handle_query()`](../src/cmd/at_cmd.c#L368) |
| Set commands (`AT+X=v`), range checks | `at_handle_set()` in [at_cmd.c](../src/cmd/at_cmd.c) |
| Execute commands (`AT+X`) | [`at_handle_execute()`](../src/cmd/at_cmd.c#L566) |
| What set/execute commands do | [`at_set_handler()`](../src/robot/robot.c#L254), [`at_exec_handler()`](../src/robot/robot.c#L305) |
| Build flags | `AT_CMD_HELP_ENABLED`, `AT_CMD_ALL_QUERY`, `AT_CMD_PID_TOGGLE`, `UART_ECHO_ENABLED` ([02](02-build-and-configuration.md#build-options)) |

## Syntax

| Format | Meaning | Example |
|--------|---------|---------|
| `AT` | Connection test | `AT` → `OK` |
| `AT+CMD?` | Query | `AT+ANGLE?` → `+ANGLE:1.23` |
| `AT+CMD=value` | Set | `AT+KP=20` → `OK` |
| `AT+CMD` | Execute | `AT+ENABLE` → `OK` |

Lines end with CR or LF. Typed characters are echoed (`UART_ECHO_ENABLED`),
and `> ` is printed when the console is ready for the next command.

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
| `AT+STREAM=0\|1` | 0 or 1 | Print `acc_deg: … \| kalman: … \| comp: …` every sample |

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

### Stream filter data for `test/statistics.py`

```
AT+STREAM=1
OK
acc_deg: 90.84 | kalman: 90.61 | comp: 90.58
acc_deg: 90.77 | kalman: 90.66 | comp: 90.59
```

Streamed angles are raw filter outputs, so upright reads about 90° (see
[05 — Sensing and IMU](05-sensing-and-imu.md)). Lines are dropped rather than
delaying the control loop when the UART queue is full, and may interleave with
command responses. Send `AT+STREAM=0` before typing other commands.

## Limitations

- No persistence: `AT+SAVE`/`AT+LOAD` return `ERROR:1`.
- `VELOCITY`/`TARGET` have no effect: there is no outer velocity loop yet
  ([06 — Control Theory](06-control-theory.md#6-cascade-control-the-next-step)).
- `AT+SPEED?` reports the last commanded values, not measured wheel speeds.
