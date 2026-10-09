# AtomS3R-CAM bridge

Firmware for an M5Stack AtomS3R-CAM (ESP32-S3, GC0308 camera, 8 MB PSRAM). It
takes the HC-06's place on the robot's Bluetooth header:

- **BLE for control.** The Nordic UART Service carries the robot's AT console to
  the phone. Every line goes to the robot unchanged, and every reply comes back.
  No pairing. It advertises as "balancing robot".
- **Wi-Fi for video only.** The camera streams MJPEG at
  `http://<ip>/stream` (320×240, compressed in software, about 10–15 frames/s).
  The app learns the URL with `AT+CAM?`.

```
phone ──BLE── Atom ──UART 115200── STM32 USART2 (Bluetooth header)
  └──── Wi-Fi (video) ────┘
```

## Robot firmware

Build the robot for the Atom, either in
[cmake/boards/f407.conf](../cmake/boards/f407.conf) (`BT_MODULE=atom`) or once:

```sh
cmake --preset f407 -DBT_MODULE=atom      # back: -DBT_MODULE=hc06, or cmake -UBT_MODULE
cmake --build --preset f407 --target flash-usb
```

`BT_MODULE=atom` sets the Bluetooth console to 115200 baud and leaves
`AT+BTNAME` out (the Atom names itself). Nothing else changes, and the HC-06
works again with `hc06`.

## Wiring

Grove cable from the Atom to the robot's 4-pin Bluetooth header, in place of the HC-06:

| Grove wire | Atom | Robot header |
|------------|------|--------------|
| Black | GND | GND |
| Red | 5V in | VCC (**measure first: must be 5 V**) |
| White | G1 = TX | the HC-06's TXD pin (→ PD6, robot RX) |
| Yellow | G2 = RX | the HC-06's RXD pin (← PD5, robot TX) |

- Both sides use 3.3 V logic.
- With Wi-Fi streaming, the Atom draws a few hundred mA at 5 V.
- If the header gives only 3.3 V, take 5 V from another pin of the board.
- To swap TX and RX, change `ATOM_ROBOT_TX_GPIO` / `ATOM_ROBOT_RX_GPIO` in menuconfig.

## Build and flash

ESP-IDF 5.2 (`~/esp/esp-idf`). The first build downloads `esp32-camera`.

```sh
cd atom
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor     # the Atom on the PC's USB
```

The Atom's own USB-C port does three jobs:
- flashing;
- the log;
- a setup console for the commands below.

To enter download mode by hand, hold the reset button for 2 s.

## Wi-Fi for the camera

The Atom joins one network in station mode, and the phone must be on the same
network. To set it, use either of these:

- the app: Settings → Camera Wi-Fi → Scan, pick the network, enter the password, Join (over BLE);
- the USB console (`idf.py monitor`): type `AT+WIFI=<ssid>,<password>`.

The Atom stores the credentials in its flash and reconnects on its own. The
menuconfig defaults (`ATOM_WIFI_SSID` / `ATOM_WIFI_PASSWORD`) apply only until
then. `sdkconfig` is git-ignored because it would hold that password.

## The Atom's own commands

The Atom answers these itself, in the robot's reply format. All other lines go
to the robot.

| Command | Reply |
|---------|-------|
| `AT+CAM?` | `+CAM:http://192.168.1.42/stream`, or `+CAM:` while Wi-Fi is down |
| `AT+WIFISCAN?` | `+WIFISCAN:-48 home\t-71 neighbour`: networks in range, strongest first, `<rssi dBm> <ssid>` separated by tabs; takes ~3 s (2.4 GHz only) |
| `AT+WIFI?` | `+WIFI:<ssid>,<off\|connecting\|connected>,<ip>` |
| `AT+WIFI=<ssid>,<password>` | `OK`, or `ERROR:3`: the SSID ends at the first comma, and a password needs 8–64 characters (empty for an open network) |
| `AT+ATOM?` | `+ATOM:<version>` |

When the phone disconnects, the Atom sends `AT+VELOCITY=0` and `AT+TURN=0`.
The robot stops driving at once, without waiting for its 1 s dead-man, and
keeps balancing.

The camera also serves `/` (a page with the video) and `/capture` (one JPEG),
which makes it easy to check from a PC browser.

## Code map

| File | What |
|------|------|
| `main/main.c` | Start-up |
| `main/ble_nus.c` | NimBLE: advertising, Nordic UART Service, notifications split to the MTU |
| `main/bridge.c` | BLE ↔ UART lines, the Atom's commands, the USB setup console |
| `main/camera_http.c` | Camera (pins from the M5Stack docs), Wi-Fi station, HTTP/MJPEG server |
| `main/Kconfig.projbuild` | BLE name, UART pins and speed, Wi-Fi defaults, JPEG quality |

## Limitations

- The BLE link has no pairing or encryption. Anyone nearby with a BLE app can
  connect and send commands, and the Wi-Fi password travels in plain text.
- The camera's orientation hasn't been set: no flip or mirror yet.
- Station mode only, with no access point of its own: there's no video away
  from the configured network.
