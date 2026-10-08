/**
 * @file bootloader.c
 * @brief Serial bootloader for the F407 board, in flash sector 0
 *
 * On every reset: start the firmware unless an update was requested
 * (AT+UPDATE), the firmware is missing or corrupt, or the host sends "BOOT"
 * within BOOT_SYNC_WINDOW_MS. Then it takes a new firmware over the console
 * UART (protocol in boot_shared.h) and starts it.
 *
 * Polled, no interrupts, no RTOS, on the 16 MHz HSI. It never erases its own
 * sector, and writes the application header only after the firmware's CRC
 * checks out, so an interrupted update leaves it waiting for the next one.
 * Motor pins stay in their reset state (inputs): nothing moves.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "board_config.h"
#include "bootloader/boot_shared.h"

#define HSI_HZ                  16000000u

#define ERASE_TIMEOUT_MS        1000    /**< Between command bytes, not for the erase itself */
#define BYTE_TIMEOUT_MS         1000

/* ==========================================================================
 * CRC-32 (zlib / IEEE 802.3, the same as Python's zlib.crc32)
 * ========================================================================== */

static uint32_t crc_table[256];

static void crc32_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc_table[i] = c;
    }
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len) {
    crc = ~crc;
    while (len--) {
        crc = crc_table[(crc ^ *data++) & 0xFFu] ^ (crc >> 8);
    }
    return ~crc;
}

/* ==========================================================================
 * Time: SysTick counts milliseconds, polled
 * ========================================================================== */

static volatile uint32_t ms_now;

static void tick_init(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(HSI_HZ / 1000u - 1u);
    systick_clear();
    systick_counter_enable();
}

static uint32_t millis(void) {
    if (systick_get_countflag()) {
        ms_now++;
    }
    return ms_now;
}

/* ==========================================================================
 * Console UART (the board's USB-serial port)
 * ========================================================================== */

static void uart_init(void) {
    rcc_periph_clock_enable(BOARD_UART_PORT_RCC);
    rcc_periph_clock_enable(BOARD_UART_RCC);

    gpio_mode_setup(BOARD_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BOARD_UART_TX_PIN);
    gpio_mode_setup(BOARD_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BOARD_UART_RX_PIN);
    gpio_set_af(BOARD_UART_PORT, BOARD_UART_AF, BOARD_UART_TX_PIN | BOARD_UART_RX_PIN);

    usart_set_baudrate(BOARD_UART, BOOT_BAUDRATE);
    usart_set_databits(BOARD_UART, 8);
    usart_set_stopbits(BOARD_UART, USART_STOPBITS_1);
    usart_set_parity(BOARD_UART, USART_PARITY_NONE);
    usart_set_flow_control(BOARD_UART, USART_FLOWCONTROL_NONE);
    usart_set_mode(BOARD_UART, USART_MODE_TX_RX);
    usart_enable(BOARD_UART);
}

/** One byte, or -1 after timeout_ms */
static int uart_getc(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        if (USART_SR(BOARD_UART) & (USART_SR_RXNE | USART_SR_ORE)) {
            return (int)(usart_recv(BOARD_UART) & 0xFFu);
        }
    }
    return -1;
}

static bool uart_read(uint8_t *buf, size_t len, uint32_t timeout_ms) {
    for (size_t i = 0; i < len; i++) {
        int c = uart_getc(timeout_ms);
        if (c < 0) {
            return false;
        }
        buf[i] = (uint8_t)c;
    }
    return true;
}

static void uart_putc(uint8_t c) {
    usart_send_blocking(BOARD_UART, c);
}

static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ==========================================================================
 * Flash
 * ========================================================================== */

/** F405/F407 sector of an address: 4 x 16K, 1 x 64K, then 128K sectors */
static uint8_t flash_sector_of(uint32_t addr) {
    uint32_t offset = addr - BOOT_FLASH_BASE;
    if (offset < 0x10000u) {
        return (uint8_t)(offset / 0x4000u);
    }
    if (offset < 0x20000u) {
        return 4;
    }
    return (uint8_t)(5u + (offset - 0x20000u) / 0x20000u);
}

static void flash_clear_errors(void) {
    /* EOP, OPERR, WRPERR, PGAERR, PGPERR, PGSERR: write 1 to clear */
    FLASH_SR = 0xF3u;
}

static bool flash_ok(void) {
    return (FLASH_SR & 0xF2u) == 0u;
}

/** Erase every sector the firmware will occupy, header included */
static bool erase_app(uint32_t size) {
    uint8_t last = flash_sector_of(APP_START_ADDR + size - 1u);

    flash_unlock();
    flash_clear_errors();
    for (uint8_t sector = flash_sector_of(APP_HEADER_ADDR); sector <= last; sector++) {
        flash_erase_sector(sector, FLASH_CR_PROGRAM_X32);
        if (!flash_ok()) {
            flash_lock();
            return false;
        }
    }
    flash_lock();
    return true;
}

/** Program and read back; len is padded to whole words with 0xFF */
static bool program(uint32_t addr, const uint8_t *data, uint32_t len) {
    flash_unlock();
    flash_clear_errors();
    for (uint32_t i = 0; i < len; i += 4u) {
        uint8_t word[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
        for (uint32_t k = 0; k < 4u && i + k < len; k++) {
            word[k] = data[i + k];
        }
        uint32_t value = le32(word);
        flash_program_word(addr + i, value);
        if (!flash_ok() || *(volatile uint32_t *)(addr + i) != value) {
            flash_lock();
            return false;
        }
    }
    flash_lock();
    return true;
}

/* ==========================================================================
 * The application
 * ========================================================================== */

static bool app_valid(void) {
    const App_Header_t *h = (const App_Header_t *)APP_HEADER_ADDR;
    if (h->magic != APP_HEADER_MAGIC || h->magic_inv != ~APP_HEADER_MAGIC ||
        h->size < 8u || h->size > APP_MAX_SIZE) {
        return false;
    }

    const uint32_t *vectors = (const uint32_t *)APP_START_ADDR;
    uint32_t sp = vectors[0];
    uint32_t pc = vectors[1];
    if (sp < 0x20000000u || sp > 0x20020000u ||
        pc < APP_START_ADDR || pc >= APP_START_ADDR + h->size || (pc & 1u) == 0u) {
        return false;
    }

    return crc32_update(0, (const uint8_t *)APP_START_ADDR, h->size) == h->crc32;
}

/** Undo everything the bootloader set up, then enter the firmware */
static void __attribute__((noreturn)) jump_to_app(void) {
    systick_counter_disable();
    STK_CVR = 0;
    usart_disable(BOARD_UART);
    rcc_periph_reset_pulse(BOARD_UART == USART3 ? RST_USART3 : RST_USART1);
    rcc_periph_clock_disable(BOARD_UART_RCC);

    const uint32_t *vectors = (const uint32_t *)APP_START_ADDR;
    uint32_t sp = vectors[0];
    uint32_t pc = vectors[1];

    SCB_VTOR = APP_START_ADDR;
    __asm__ volatile ("msr msp, %0\n\tbx %1" : : "r"(sp), "r"(pc));
    for (;;) {
    }
}

/* ==========================================================================
 * Protocol
 * ========================================================================== */

/** True once "BOOT" has arrived within timeout_ms */
static bool wait_sync(uint32_t timeout_ms) {
    static const uint8_t sync[] = { 'B', 'O', 'O', 'T' };
    size_t matched = 0;
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        int c = uart_getc(1);
        if (c < 0) {
            continue;
        }
        matched = (c == sync[matched]) ? matched + 1u : (c == sync[0] ? 1u : 0u);
        if (matched == sizeof(sync)) {
            return true;
        }
    }
    return false;
}

static void serve_update(void) {
    static uint8_t block[BOOT_BLOCK_SIZE];
    uint32_t size = 0, crc = 0, written = 0;
    bool erased = false;

    uart_putc(BOOT_ACK);

    for (;;) {
        int cmd = uart_getc(0xFFFFFFFFu);
        uint8_t hdr[8];

        switch (cmd) {
        case 'W':
            if (!uart_read(hdr, 8, ERASE_TIMEOUT_MS)) {
                uart_putc(BOOT_NACK);
                break;
            }
            size = le32(&hdr[0]);
            crc = le32(&hdr[4]);
            written = 0;
            erased = size >= 8u && size <= APP_MAX_SIZE && erase_app(size);
            uart_putc(erased ? BOOT_ACK : BOOT_NACK);
            break;

        case 'D': {
            uint8_t lenb[2], crcb[4];
            if (!uart_read(lenb, 2, BYTE_TIMEOUT_MS)) {
                uart_putc(BOOT_NACK);
                break;
            }
            uint32_t len = (uint32_t)lenb[0] | ((uint32_t)lenb[1] << 8);
            bool ok = len > 0u && len <= BOOT_BLOCK_SIZE &&
                      uart_read(block, len, BYTE_TIMEOUT_MS) &&
                      uart_read(crcb, 4, BYTE_TIMEOUT_MS);
            ok = ok && erased && written + len <= size &&
                 crc32_update(0, block, len) == le32(crcb) &&
                 program(APP_START_ADDR + written, block, len);
            if (ok) {
                written += len;
            }
            uart_putc(ok ? BOOT_ACK : BOOT_NACK);
            break;
        }

        case 'E': {
            bool ok = erased && written == size &&
                      crc32_update(0, (const uint8_t *)APP_START_ADDR, size) == crc;
            if (ok) {
                App_Header_t h = { APP_HEADER_MAGIC, size, crc, ~APP_HEADER_MAGIC };
                ok = program(APP_HEADER_ADDR, (const uint8_t *)&h, sizeof(h)) && app_valid();
            }
            uart_putc(ok ? BOOT_ACK : BOOT_NACK);
            break;
        }

        case 'G':
            uart_putc(BOOT_ACK);
            while (!usart_get_flag(BOARD_UART, USART_SR_TC)) {
            }
            scb_reset_system();
            break;

        case 'B':
            /* A repeated "BOOT" from a host that missed our ACK */
            uart_putc(BOOT_ACK);
            break;

        default:
            break;
        }
    }
}

int main(void) {
    bool requested = (BOOT_FLAG == BOOT_FLAG_UPDATE);
    BOOT_FLAG = 0;

    crc32_init();
    tick_init();
    uart_init();

    bool valid = app_valid();

    if (!requested && valid && !wait_sync(BOOT_SYNC_WINDOW_MS)) {
        jump_to_app();
    }

    /* Requested, recovery, or no valid firmware: wait for the host */
    if (!requested && valid) {
        serve_update();             /* "BOOT" already received */
    }
    while (!wait_sync(1000)) {
    }
    serve_update();
    return 0;
}
