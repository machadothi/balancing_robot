#!/usr/bin/env python3
"""Prefix a firmware .bin with the bootloader's application header.

The result is what the bootloader writes at 0x08004000 after an update:
a 512-byte header (magic, size, CRC-32, ~magic, then 0xFF) followed by the
firmware. Flashing it with the ST-Link gives a firmware the bootloader starts.

Usage: make_app_image.py firmware.bin firmware.img
"""

import struct
import sys
import zlib

APP_HEADER_SIZE = 0x200
APP_HEADER_MAGIC = 0x31505041          # "APP1", src/bootloader/boot_shared.h


def main() -> None:
    src, dst = sys.argv[1], sys.argv[2]
    firmware = open(src, "rb").read()
    header = struct.pack("<4I", APP_HEADER_MAGIC, len(firmware), zlib.crc32(firmware),
                         ~APP_HEADER_MAGIC & 0xFFFFFFFF)
    with open(dst, "wb") as f:
        f.write(header.ljust(APP_HEADER_SIZE, b"\xff") + firmware)


if __name__ == "__main__":
    main()
