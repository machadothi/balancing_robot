/**
 * @file boot_shared.h
 * @brief What the F407 bootloader, the robot firmware and scripts/flash_usb.py
 *        agree on: flash layout, update request and serial protocol
 */

#ifndef BOOT_SHARED_H
#define BOOT_SHARED_H

#include <stdint.h>

/* ==========================================================================
 * Flash layout (src/stm32f407vet6_app.ld, src/bootloader/bootloader_f407.ld)
 * ========================================================================== */

#define BOOT_FLASH_BASE         0x08000000u
#define BOOT_FLASH_SIZE         0x80000u        /**< 512 KB */
#define BOOT_SIZE               0x4000u         /**< Sector 0: the bootloader */

/** Application header, then the firmware 512 bytes later (VTOR alignment) */
#define APP_HEADER_ADDR         (BOOT_FLASH_BASE + BOOT_SIZE)
#define APP_HEADER_SIZE         0x200u
#define APP_START_ADDR          (APP_HEADER_ADDR + APP_HEADER_SIZE)
#define APP_MAX_SIZE            (BOOT_FLASH_SIZE - BOOT_SIZE - APP_HEADER_SIZE)

#define APP_HEADER_MAGIC        0x31505041u     /**< "APP1" */

/** Written last, after the firmware is verified: no header, no start */
typedef struct {
    uint32_t magic;             /**< APP_HEADER_MAGIC */
    uint32_t size;              /**< Firmware bytes from APP_START_ADDR */
    uint32_t crc32;             /**< CRC-32 (zlib) of those bytes */
    uint32_t magic_inv;         /**< ~APP_HEADER_MAGIC */
} App_Header_t;

/* ==========================================================================
 * Update request: firmware -> bootloader across a software reset
 *
 * The last 16 bytes of SRAM are outside both linker scripts' `ram`, so
 * neither program's stack or data touches them, and a reset keeps SRAM.
 * ========================================================================== */

#define BOOT_FLAG_ADDR          0x2001FFF0u
#define BOOT_FLAG_UPDATE        0xB007F1A6u     /**< AT+UPDATE: stay in the bootloader */
#define BOOT_FLAG               (*(volatile uint32_t *)BOOT_FLAG_ADDR)

/* ==========================================================================
 * Serial protocol on the console UART, 115200 8N1 (16 MHz HSI, no PLL)
 *
 *   host                                   bootloader
 *   "BOOT"                              -> ACK
 *   'W' size:u32 crc32:u32              -> erases the sectors needed, ACK
 *   'D' len:u16 data[len] crc32:u32     -> programs and reads back, ACK
 *   ... (len <= BOOT_BLOCK_SIZE, a multiple of 4 except for the last)
 *   'E'                                 -> checks the CRC of the firmware,
 *                                          writes the header, ACK
 *   'G'                                 -> resets into the firmware
 *
 * Integers are little-endian. Any failure answers NACK; the host may resend
 * a 'D' block or start over with 'W'. After every reset the bootloader
 * listens for "BOOT" for BOOT_SYNC_WINDOW_MS before starting the firmware.
 * ========================================================================== */

#define BOOT_BAUDRATE           115200
#define BOOT_SYNC_WINDOW_MS     200
#define BOOT_BLOCK_SIZE         256
#define BOOT_ACK                0x79
#define BOOT_NACK               0x1F

#endif // BOOT_SHARED_H
