#!/usr/bin/env python3
"""Update the robot firmware over the USB console, through the bootloader.

1. Sends AT+UPDATE to the running firmware (921600 baud): it restarts into
   the bootloader. Without a running firmware the bootloader is already
   waiting; if the firmware runs but does not answer, use --power-cycle and
   switch the board off and on: the bootloader listens for 200 ms at reset.
2. Talks to the bootloader at 115200 baud: erase, 256-byte blocks with a
   CRC each, then a CRC check of the whole firmware before it is marked valid.
3. Starts the new firmware and checks that it answers.

Protocol: src/bootloader/boot_shared.h. Nothing is marked valid until the
whole firmware checks out, so an interrupted update can simply be rerun.

Usage: flash_usb.py [--port /dev/ttyACM0] firmware.bin
"""

import argparse
import struct
import sys
import time
import zlib

import serial

CONSOLE_BAUD = 921600
BOOT_BAUD = 115200
ACK, NACK = 0x79, 0x1F
BLOCK = 256


def open_port(port: str, baud: int) -> serial.Serial:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.1
    # Keep DTR/RTS released: on this board they can reset the MCU
    s.dtr = s.rts = False
    s.open()
    return s


def request_update(port: str) -> None:
    with open_port(port, CONSOLE_BAUD) as s:
        s.reset_input_buffer()
        s.write(b"AT+UPDATE\r")
        reply = s.read(64)
    print("AT+UPDATE ->", "OK" if b"OK" in reply else f"no reply ({reply!r}); trying the bootloader anyway")


def wait_ack(s: serial.Serial, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = s.read(1)
        if b:
            if b[0] == ACK:
                return True
            if b[0] == NACK:
                return False
    raise TimeoutError("no answer from the bootloader")


def sync(s: serial.Serial, timeout: float) -> None:
    """Send "BOOT" every 20 ms until the bootloader acknowledges it."""
    deadline = time.monotonic() + timeout
    s.timeout = 0.02
    while time.monotonic() < deadline:
        s.write(b"BOOT")
        reply = s.read(16)
        if bytes([ACK]) in reply:
            s.timeout = 0.1
            time.sleep(0.05)
            s.reset_input_buffer()      # ACKs to the extra "BOOT"s
            return
    raise SystemExit("bootloader not found: is it installed (flash-bootloader)? "
                     "If the firmware hangs, rerun with --power-cycle")


def upload(s: serial.Serial, firmware: bytes) -> None:
    s.write(b"W" + struct.pack("<II", len(firmware), zlib.crc32(firmware)))
    print(f"erasing for {len(firmware)} bytes ...", flush=True)
    if not wait_ack(s, 20.0):
        raise SystemExit("erase refused (size?)")

    for offset in range(0, len(firmware), BLOCK):
        block = firmware[offset:offset + BLOCK]
        packet = b"D" + struct.pack("<H", len(block)) + block + struct.pack("<I", zlib.crc32(block))
        for attempt in range(3):
            s.write(packet)
            if wait_ack(s, 2.0):
                break
            # A failed block was not counted by the bootloader: resend it
        else:
            raise SystemExit(f"block at {offset} failed three times")
        done = offset + len(block)
        if done % (BLOCK * 32) == 0 or done == len(firmware):
            print(f"  {done * 100 // len(firmware):3d} %  {done}/{len(firmware)} bytes", flush=True)

    s.write(b"E")
    if not wait_ack(s, 5.0):
        raise SystemExit("final CRC check failed: the firmware was NOT marked valid; run again")
    print("verified, starting the firmware")
    s.write(b"G")
    wait_ack(s, 1.0)


def check_running(port: str) -> None:
    time.sleep(1.5)
    with open_port(port, CONSOLE_BAUD) as s:
        s.reset_input_buffer()
        s.write(b"AT+VERSION?\r")
        reply = s.read(200)
    print("firmware answers:", reply.decode(errors="replace").split("\r\n")[1:2] or reply)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("firmware")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--no-request", action="store_true",
                        help="skip AT+UPDATE (the bootloader is already waiting)")
    parser.add_argument("--power-cycle", action="store_true",
                        help="recovery: wait up to 60 s while you switch the board off and on")
    args = parser.parse_args()

    firmware = open(args.firmware, "rb").read()
    start = time.monotonic()
    if args.power_cycle:
        print("switch the board off and on now ...", flush=True)
    elif not args.no_request:
        request_update(args.port)
        time.sleep(0.3)
    with open_port(args.port, BOOT_BAUD) as s:
        sync(s, 60.0 if args.power_cycle else 5.0)
        print("bootloader ready")
        upload(s, firmware)
    check_running(args.port)
    print(f"done in {time.monotonic() - start:.1f} s")


if __name__ == "__main__":
    try:
        main()
    except (TimeoutError, serial.SerialException) as exc:
        sys.exit(f"error: {exc}")
