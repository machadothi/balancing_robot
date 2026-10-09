# Balance Bot app

Android app that drives and tunes the robot over its Bluetooth console: an
HC-06 (classic Bluetooth serial, 9600 baud) or the AtomS3R-CAM bridge (BLE, plus
camera video over Wi-Fi, [atom/](../atom/README.md)), whichever the robot has. It speaks the AT commands of
[docs/10](../docs/10-at-commands.md); nothing robot-specific lives in the app
except the parameter list in `model/RobotParam.kt`.

| Tab | What it does |
|-----|--------------|
| Connect | Atom: BLE scan, no pairing. HC-06: paired devices (pair "balancing robot" first, PIN 1234). Connect / disconnect |
| Drive | Camera (Atom only, once its Wi-Fi is up), joystick (forward/back = `AT+VELOCITY`, left/right = `AT+TURN`), max speed and turn, Enable / Stop, tilt and speed |
| Live | Status, tilt, speed, balance target, motor output, encoders; charts of the last minute |
| Settings | Every tunable value (`AT+KP` ... `AT+DEADBAND`), read from the robot, range-checked before sending; the Atom's camera Wi-Fi (`AT+WIFI=`) |

STOP is on every screen once connected.

**Safety.** The jog only sends while "Drive with the stick" is on, and repeats
its command every 300 ms. The firmware zeroes speed and turn 1 s after the last
`AT+VELOCITY`/`AT+TURN`, so a lost link, a closed app or a locked phone stops
the robot by itself.

**Data.** The app polls `AT+LIVE?` about 5 times a second (one short reply with
everything the screens show); the USB telemetry stream is far too fast for
9600 baud. The firmware needs `AT+LIVE?`: flash it first.

**Permissions.** "Nearby devices" (Bluetooth connect and scan); on Android 11
and older, location instead, which BLE scans need there. The camera uses the
network: on GrapheneOS, the app's **Network** permission must be on.

## Build and install

Same stack as `~/git/app_playground`: Kotlin, Jetpack Compose, Hilt, navigation.

```sh
cd android
export JAVA_HOME=~/tools/jdk-21.0.12.1+1     # any JDK 17+
./gradlew assembleDebug testDebugUnitTest
~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk
```

`local.properties` (git-ignored) points to the Android SDK: `sdk.dir=/home/<you>/Android/Sdk`.

## Code map

| Layer | Files |
|-------|-------|
| Protocol | `data/at/AtProtocol.kt` (reply parsing), `data/at/AtSession.kt` (one command at a time) |
| Link | `data/link/RobotLink.kt` (a byte stream), `BluetoothTransport.kt` (HC-06, SPP socket), `BleTransport.kt` (Atom, Nordic UART Service) |
| Camera | `data/camera/MjpegReader.kt` (splits the MJPEG stream), `ui/components/CameraView.kt` |
| State | `repository/RobotRepositoryImpl.kt`: connection, polling, history, jog, parameters |
| Screens | `ui/screen/*`: one ViewModel + Composable per tab |
| Tests | `app/src/test`: reply parsing, `AT+LIVE?` format, parameter ranges, joystick mapping, MJPEG framing |
