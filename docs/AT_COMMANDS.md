# AT Command Reference

This document describes the AT command interface for the Self-Balancing Robot.

## Overview

The AT command interface provides serial communication control over UART at **921600 baud**. Commands follow the standard AT command format and are **case-insensitive**.

### Command Syntax

| Format | Description | Example |
|--------|-------------|---------|
| `AT` | Test connection | `AT` → `OK` |
| `AT+CMD?` | Query value | `AT+ANGLE?` → `+ANGLE:12.34` |
| `AT+CMD=value` | Set value | `AT+KP=1.5` → `OK` |
| `AT+CMD` | Execute action | `AT+ENABLE` → `OK` |

### Response Format

| Response | Meaning |
|----------|---------|
| `OK` | Command executed successfully |
| `ERROR:N` | Error with code N |
| `+CMD:value` | Query response with data |

### Error Codes

| Code | Meaning |
|------|---------|
| 0 | Generic error |
| 1 | Unknown command |
| 2 | Invalid parameter |
| 3 | Value out of range |
| 4 | System not ready |

---

## Commands

### System Commands

#### AT - Test Connection
```
AT
OK
```
Tests if the device is responding.

#### AT+VERSION? - Firmware Version
```
AT+VERSION?
+VERSION:1.0.0
OK
```
Returns the firmware version string.

#### AT+STATUS? - Robot Status
```
AT+STATUS?
+STATUS:DISABLED,UNBALANCED
OK
```
Returns motor state (`ENABLED`/`DISABLED`) and balance state (`BALANCED`/`UNBALANCED`).

#### AT+RESET - System Reset
```
AT+RESET
OK
```
Performs a software reset of the microcontroller.

#### AT+HELP - Show Help
```
AT+HELP
```
Displays a summary of all available commands.

---

### Sensor Queries

#### AT+ALL? - All Sensor Data
```
AT+ALL?
+ALL:0.123,-0.045,1.001,0.500,-0.300,0.100,2.50
OK
```
Returns all sensor data in CSV format:
`acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, angle`

#### AT+ACC_X? / AT+ACC_Y? / AT+ACC_Z? - Accelerometer
```
AT+ACC_X?
+ACC_X:0.123
OK
```
Returns accelerometer reading in g (3 decimal places).

#### AT+GYRO_X? / AT+GYRO_Y? / AT+GYRO_Z? - Gyroscope
```
AT+GYRO_X?
+GYRO_X:0.500
OK
```
Returns gyroscope reading in degrees/second (3 decimal places).

#### AT+ANGLE? - Tilt Angle
```
AT+ANGLE?
+ANGLE:2.50
OK
```
Returns the filtered tilt angle in degrees (2 decimal places).

---

### Motion Control

#### AT+ENABLE - Enable Motors
```
AT+ENABLE
OK
```
Enables motor control. Robot will attempt to balance.

#### AT+DISABLE - Disable Motors
```
AT+DISABLE
OK
```
Disables motor control. Motors will coast to a stop.

#### AT+STOP - Emergency Stop
```
AT+STOP
OK
```
Immediately stops all motors (emergency brake).

#### AT+VELOCITY? - Query Velocity
```
AT+VELOCITY?
+VELOCITY:0.00
OK
```
Returns current velocity estimate (2 decimal places).

#### AT+VELOCITY=n / AT+TARGET=n - Set Target Velocity
```
AT+VELOCITY=50
OK
```
Sets target forward/backward velocity.
- **Range:** -100 to 100
- Positive = forward, Negative = backward

#### AT+TURN=n - Set Turn Rate
```
AT+TURN=-30
OK
```
Sets the turn rate for differential steering.
- **Range:** -100 to 100
- Positive = turn right, Negative = turn left

#### AT+TURN? - Query Turn Rate
```
AT+TURN?
+TURN:-30.00
OK
```

#### AT+SPEED=left,right - Set Wheel Speeds
```
AT+SPEED=-50,50
OK
```
Sets individual wheel speeds directly.
- **Range:** -100 to 100 for each wheel
- Useful for manual control or testing

#### AT+SPEED? - Query Wheel Speeds
```
AT+SPEED?
+SPEED:-50.0,50.0
OK
```

---

### PID Tuning

#### AT+KP? / AT+KP=n - Proportional Gain
```
AT+KP?
+KP:1.5000
OK

AT+KP=2.0
OK
```
Query or set the PID proportional gain.
- **Range:** ≥ 0

#### AT+KI? / AT+KI=n - Integral Gain
```
AT+KI?
+KI:0.0100
OK

AT+KI=0.02
OK
```
Query or set the PID integral gain.
- **Range:** ≥ 0

#### AT+KD? / AT+KD=n - Derivative Gain
```
AT+KD?
+KD:0.5000
OK

AT+KD=0.8
OK
```
Query or set the PID derivative gain.
- **Range:** ≥ 0

---

### Configuration Persistence

#### AT+SAVE - Save Settings
```
AT+SAVE
OK
```
Saves current PID parameters to flash memory.

#### AT+LOAD - Load Settings
```
AT+LOAD
OK
```
Loads PID parameters from flash memory.

#### AT+DEFAULT - Reset to Defaults
```
AT+DEFAULT
OK
```
Resets all parameters to factory defaults.

---

## Examples

### Basic Test Sequence
```
AT
OK
AT+VERSION?
+VERSION:1.0.0
OK
AT+STATUS?
+STATUS:DISABLED,UNBALANCED
OK
```

### Enable and Control
```
AT+ENABLE
OK
AT+VELOCITY=30
OK
AT+TURN=10
OK
AT+STOP
OK
AT+DISABLE
OK
```

### PID Tuning Session
```
AT+KP?
+KP:1.5000
OK
AT+KP=2.0
OK
AT+KI=0.01
OK
AT+KD=0.5
OK
AT+SAVE
OK
```

### Monitor Sensors
```
AT+ANGLE?
+ANGLE:1.23
OK
AT+ALL?
+ALL:0.012,-0.034,0.998,0.123,-0.456,0.789,1.23
OK
```

### Direct Wheel Control
```
AT+SPEED=50,50
OK
AT+SPEED=-30,30
OK
AT+SPEED=0,0
OK
```

---

## Notes

1. **Baud Rate:** 921600 bps, 8N1
2. **Line Ending:** CR (`\r`) or LF (`\n`)
3. **Echo:** Characters are echoed as typed
4. **Prompt:** `> ` indicates ready for input
5. **Case:** Commands are case-insensitive (`at+help` = `AT+HELP`)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No response | Check baud rate (921600) |
| `ERROR:4` | Robot state not initialized, wait for startup |
| `ERROR:1` | Unknown command, check syntax with `AT+HELP` |
| `ERROR:3` | Value out of range, check parameter limits |
