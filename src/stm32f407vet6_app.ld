/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Linker script for ST STM32F407VET6: robot firmware behind the bootloader.
 *
 * 0x08000000  16K  bootloader (sector 0)
 * 0x08004000  512  application header (magic, size, CRC32)
 * 0x08004200       firmware: vector table first, 512-byte aligned for VTOR
 */

/* 64K CCM RAM at 0x10000000 is left out: DMA cannot access it. */
/* The last 16 bytes of SRAM (0x2001FFF0) are left out of `ram`: the robot
 * firmware and the bootloader pass the update request there, untouched by
 * stack, .data and .bss (src/bootloader/boot_shared.h). */
MEMORY
{
	rom (rx) : ORIGIN = 0x08004200, LENGTH = 512K - 16K - 512
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K - 16
}

INCLUDE stm32f4_sections.ld
