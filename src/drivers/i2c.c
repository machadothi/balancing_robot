/**
 * @file i2c.c
 * @brief I2C driver for STM32F1 and STM32F4
 * 
 * Features:
 *   - Master mode only
 *   - Polling (blocking)
 *   - Interrupt-driven (non-blocking)
 *   - DMA-driven (hardware transfers)
 *   - Bus recovery for stuck slaves
 *   - Configurable timeout
 * 
 * Hardware (peripheral, pins and DMA streams from board_config.h):
 *   - BOARD_I2C on BOARD_I2C_SCL_PIN / BOARD_I2C_SDA_PIN
 *   - BOARD_I2C_DMA_TX / BOARD_I2C_DMA_RX: DMA channels (F1) or streams (F4)
 *   - 100 kHz standard mode
 * 
 * @author Thiago Cunha
 * @date 2023
 */

#include "config.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#if I2C_DMA_ENABLED
#include <libopencm3/stm32/dma.h>
#endif
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>

#include "board_config.h"
#include "i2c.h"
#include "drivers/gpio_compat.h"
#include "log/log.h"

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define I2C_SCL_PIN     BOARD_I2C_SCL_PIN
#define I2C_SDA_PIN     BOARD_I2C_SDA_PIN
#define I2C_PORT        BOARD_I2C_PORT
#define I2C_PERIPH      BOARD_I2C

/** Upper bound for the BTF poll in the DMA TX ISR (~100 µs at 100 kHz) */
#define I2C_ISR_BTF_SPINS   100000U

#if I2C_DMA_ENABLED
#if defined(STM32F1)
#define I2C_DMA_PSIZE_8BIT  DMA_CCR_PSIZE_8BIT
#define I2C_DMA_MSIZE_8BIT  DMA_CCR_MSIZE_8BIT
#define I2C_DMA_PL_HIGH     DMA_CCR_PL_HIGH

static void i2c_dma_disable(uint8_t channel) {
    dma_disable_channel(BOARD_I2C_DMA, channel);
}

static void i2c_dma_enable(uint8_t channel) {
    dma_enable_channel(BOARD_I2C_DMA, channel);
}
#else
#define I2C_DMA_PSIZE_8BIT  DMA_SxCR_PSIZE_8BIT
#define I2C_DMA_MSIZE_8BIT  DMA_SxCR_MSIZE_8BIT
#define I2C_DMA_PL_HIGH     DMA_SxCR_PL_HIGH

/* A stream must read back as disabled before it can be reconfigured */
static void i2c_dma_disable(uint8_t stream) {
    dma_disable_stream(BOARD_I2C_DMA, stream);
    while (DMA_SCR(BOARD_I2C_DMA, stream) & DMA_SxCR_EN);
}

/* Stale flags from the previous transfer must be cleared before enabling */
static void i2c_dma_enable(uint8_t stream) {
    dma_clear_interrupt_flags(BOARD_I2C_DMA, stream,
        DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
    dma_enable_stream(BOARD_I2C_DMA, stream);
}
#endif // defined(STM32F1)
#endif // I2C_DMA_ENABLED

#define NO_OPT __attribute__((optimize("O0")))
#define systicks    xTaskGetTickCount

/* Temp buffer for register address in async operations */
static uint8_t i2c_reg_addr_buf;

/* ==========================================================================
 * Private Function Prototypes
 * ========================================================================== */

/**
 * @brief Converts I2C_Fails_t enum to string.
 * 
 * @param fail I2C_Fails_t enum value
 * @return const char* String representation of the I2C_Fails_t
 */
const char* i2c_error_string(I2C_Fails_t fail) {
    switch (fail) {
        case I2C_Ok:
            return "I2C_Ok";
        case I2C_Addr_Timeout:
            return "I2C_Addr_Timeout";
        case I2C_Write_Timeout:
            return "I2C_Write_Timeout";
        case I2C_Read_Timeout:
            return "I2C_Read_Timeout";
        case I2C_Busy_Timeout:
            return "I2C_Busy_Timeout";
        case I2C_Busy:
            return "I2C_Busy";
        case I2C_DMA_Error:
            return "I2C_DMA_Error";
        case I2C_Bus_Error:
            return "I2C_Bus_Error";
        case I2C_Arbitration_Lost:
            return "I2C_Arbitration_Lost";
        case I2C_Nack:
            return "I2C_Nack";
        case I2C_Overrun:
            return "I2C_Overrun";
        default:
            return "UNKNOWN_ERROR";
    }
}

/* Keep old name for internal use */
static const char* i2c_fail_to_string(I2C_Fails_t fail) {
    return i2c_error_string(fail);
}


/**
 * Run a write/read transaction to a given 7bit i2c address
 * If both write & read are provided, the read will use repeated start.
 * Both write and read are optional
 * There are likely still issues with repeated start/stop condtions!
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param w buffer of data to write
 * @param wn length of w
 * @param r destination buffer to read into
 * @param rn number of bytes to read
 */
static I2C_Fails_t i2c_transfer(I2C_Control_t *dev, const uint8_t *w, size_t wn,
  uint8_t *r, size_t rn);

/**
 * @brief Reads a given number of bytes from I2C bus. The STM32F1 has different
 * conditions for reading when n = 1, n = 2 and n > 2.
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param res destination buffer to read into
 * @param n number of bytes to read
 */
static I2C_Fails_t i2c_read(I2C_Control_t *dev, uint8_t *res, size_t n);

/**
 * @brief writes data into I2C bus.
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param data content to write
 * @param n number of bytes to write 
 */
static I2C_Fails_t i2c_write(I2C_Control_t *dev, const uint8_t *data, size_t n);

/**
 * @brief Compute the difference in ticks (handles wrap-around)
 */
static inline TickType_t diff_ticks(TickType_t early, TickType_t later) {
    if (later >= early)
        return later - early;
    return ~(TickType_t)0 - early + 1 + later;
}

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Recover I2C bus from stuck state
 * 
 * If a slave is holding SDA low (e.g., interrupted mid-transaction),
 * toggle SCL up to 9 times to release it.
 */
#if I2C_BUS_RECOVERY
void i2c_bus_recovery(void) {
    /* Configure SCL as GPIO output */
    gpio_compat_output(I2C_PORT, I2C_SCL_PIN, true);
    
    /* Toggle SCL up to 9 times */
    for (int i = 0; i < 9; i++) {
        gpio_clear(I2C_PORT, I2C_SCL_PIN);
        for (volatile int j = 0; j < 1000; j++);
        gpio_set(I2C_PORT, I2C_SCL_PIN);
        for (volatile int j = 0; j < 1000; j++);
        
        /* Check if SDA is released */
        if (gpio_get(I2C_PORT, I2C_SDA_PIN))
            break;
    }
    
    /* Reconfigure as I2C alternate function */
    gpio_compat_af_output(I2C_PORT, I2C_SCL_PIN, BOARD_I2C_AF, true);
}
#endif

void NO_OPT i2c_setup_peripheral(void) {
    rcc_periph_clock_enable(BOARD_I2C_PORT_RCC);
    rcc_periph_clock_enable(BOARD_I2C_RCC);
    
    /* Configure I2C pins as open-drain alternate function */
    gpio_compat_af_output(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN, BOARD_I2C_AF, true);
    
    /* Set idle high */
    gpio_set(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
    
#if I2C_BUS_RECOVERY
    /* Recover bus if stuck */
    i2c_bus_recovery();
#endif
}

// -----------------------------------------------------------------------------

I2C_Fails_t NO_OPT i2c_configure(I2C_Control_t *dev, uint32_t i2c, 
                                 uint8_t address, uint32_t timeout) {
    dev->device = i2c;
    dev->addr = address;
    dev->timeout = timeout;

    log_message(INFO, I2C_BUS, "Setting up I2C.");

    /* 
     * STM32F1 I2C BUSY flag errata workaround:
     * The BUSY flag can get stuck. To fix:
     * 1. Disable I2C peripheral
     * 2. Configure SCL/SDA as GPIO outputs
     * 3. Toggle SCL to release any stuck slave
     * 4. Generate STOP condition manually
     * 5. Reconfigure as I2C alternate function
     * 6. Reset I2C via SWRST
     */
    
    /* Step 1: Disable I2C */
    i2c_peripheral_disable(dev->device);
    
    /* Step 2-4: Bus recovery with STOP condition */
    /* Configure as GPIO */
    gpio_compat_output(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN, true);
    
    /* Ensure both lines are high */
    gpio_set(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
    for (volatile int i = 0; i < 1000; i++);
    
    /* Toggle SCL to release stuck slave */
    for (int i = 0; i < 16; i++) {
        gpio_clear(I2C_PORT, I2C_SCL_PIN);
        for (volatile int j = 0; j < 500; j++);
        gpio_set(I2C_PORT, I2C_SCL_PIN);
        for (volatile int j = 0; j < 500; j++);
    }
    
    /* Generate STOP: SDA low while SCL high, then SDA high */
    gpio_clear(I2C_PORT, I2C_SDA_PIN);
    for (volatile int i = 0; i < 500; i++);
    gpio_set(I2C_PORT, I2C_SCL_PIN);
    for (volatile int i = 0; i < 500; i++);
    gpio_set(I2C_PORT, I2C_SDA_PIN);
    for (volatile int i = 0; i < 500; i++);
    
    /* Step 5: Reconfigure as I2C */
    gpio_compat_af_output(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN, BOARD_I2C_AF, true);
    
    /* Step 6: Software reset via SWRST bit */
    I2C_CR1(dev->device) |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 100; i++);
    I2C_CR1(dev->device) &= ~I2C_CR1_SWRST;
    
    /* Configure I2C peripheral */
    i2c_set_speed(dev->device, i2c_speed_sm_100k, rcc_apb1_frequency / 1000000);
    i2c_peripheral_enable(dev->device);

    /* Final check */
    if ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
        log_message(ERROR, I2C_BUS, "I2C bus busy after configure");
        return I2C_Busy_Timeout;
    }

    return I2C_Ok;
}

I2C_Fails_t NO_OPT i2c_read_bit(I2C_Control_t *dev, uint8_t regAddr, 
                                 uint8_t bitNum, uint8_t *data) {
    uint8_t b = 0;

    I2C_Fails_t status = i2c_read_byte(dev, regAddr, &b);
    if (status != I2C_Ok) {
        return status;
    }

    *data = (b >> bitNum) & 0x01;
    return I2C_Ok;
}

I2C_Fails_t NO_OPT i2c_write_bit(I2C_Control_t *dev, uint8_t regAddr, 
                                  uint8_t bitNum, uint8_t data) {
    uint8_t b = 0;

    I2C_Fails_t status = i2c_read_byte(dev, regAddr, &b);
    if (status != I2C_Ok) {
        return status;
    }

    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_write_byte(dev, regAddr, b);
}

I2C_Fails_t NO_OPT i2c_write_bits(I2C_Control_t *dev, uint8_t regAddr, 
                                   uint8_t bitStart, uint8_t length, uint8_t data) {
    uint8_t b = 0;

    I2C_Fails_t status = i2c_read_byte(dev, regAddr, &b);
    if (status != I2C_Ok) {
        return status;
    }

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    b &= ~mask;
    b |= data;

    return i2c_write_byte(dev, regAddr, b);
}

I2C_Fails_t NO_OPT i2c_write_byte(I2C_Control_t *dev, uint8_t regAddr, uint8_t data) {
    const uint8_t content[2] = {regAddr, data};
    return i2c_transfer(dev, content, 2, NULL, 0);
}

I2C_Fails_t i2c_read_byte(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data) {
    return i2c_transfer(dev, &regAddr, 1, data, 1);
}

I2C_Fails_t i2c_read_bytes(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data, 
                           uint8_t length) {
    return i2c_transfer(dev, &regAddr, 1, data, length);
}

// -----------------------------------------------------------------------------
/* ---------------------------- PRIVATE FUNCTIONS ----------------------------*/
// -----------------------------------------------------------------------------

/**
 * @brief Poll SR1 for a flag, bounded by the device timeout
 *
 * Framing steps (START, address, one byte) finish in well under a millisecond,
 * so polling beats an interrupt round-trip. A stuck bus or an absent device
 * must still never hang the calling task.
 *
 * @return I2C_Ok, I2C_Nack if the slave did not acknowledge, or on_timeout
 */
static I2C_Fails_t i2c_wait_sr1(I2C_Control_t *dev, uint32_t flag, TickType_t t0,
                                I2C_Fails_t on_timeout) {
    while (!(I2C_SR1(dev->device) & flag)) {
        if (I2C_SR1(dev->device) & I2C_SR1_AF) {
            I2C_SR1(dev->device) &= ~I2C_SR1_AF;
            return I2C_Nack;
        }
        if (diff_ticks(t0, systicks()) > dev->timeout) {
            return on_timeout;
        }
    }
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

static I2C_Fails_t NO_OPT
i2c_write(I2C_Control_t *dev, const uint8_t *data, size_t n)
{
    TickType_t t0 = systicks();

    while ((I2C_SR2(dev->device) & I2C_SR2_BUSY)) {
        if ( diff_ticks(t0,systicks()) > dev->timeout ) {
            log_message(ERROR, I2C_BUS,"I2C BUSY TIMEOUT!");
            return I2C_Busy_Timeout;
        }
    }

    i2c_send_start(dev->device);

    /* Wait for the end of the start condition, master mode selected, 
        and BUSY bit set */
    if (i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout) != I2C_Ok) {
        log_message(ERROR, I2C_BUS, "I2C START TIMEOUT!");
        i2c_send_stop(dev->device);
        return I2C_Busy_Timeout;
    }

    i2c_send_7bit_address(dev->device, dev->addr, I2C_WRITE);

    /* Waiting for address is transferred. */
    while (!(I2C_SR1(dev->device) & I2C_SR1_ADDR)) {
        if ( diff_ticks(t0,systicks()) > dev->timeout ) {
            log_message(ERROR, I2C_BUS,"I2C ADDRESS ACK TIMEOUT!");
            return I2C_Addr_Timeout;
        }
    }

    /* Clearing ADDR condition sequence. */
    (void)I2C_SR2(dev->device);

    for (size_t i = 0; i < n; i++) {
        i2c_send_data(dev->device, data[i]);
        while (!(I2C_SR1(dev->device) & (I2C_SR1_BTF))) {
            if ( diff_ticks(t0,systicks()) > dev->timeout ) {
                log_message(ERROR, I2C_BUS,"I2C WRITE TIMEOUT!");
                return I2C_Write_Timeout;
            }
        }
    }

    return I2C_Ok;
}

// -----------------------------------------------------------------------------

static I2C_Fails_t NO_OPT
i2c_read(I2C_Control_t *dev, uint8_t *res, size_t n)
{
    TickType_t t0 = systicks();

    i2c_send_start(dev->device);
    i2c_enable_ack(dev->device);

    if (i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout) != I2C_Ok) {
        log_message(ERROR, I2C_BUS, "I2C START TIMEOUT!");
        i2c_send_stop(dev->device);
        return I2C_Busy_Timeout;
    }

    i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);

    /* Waiting for address is transferred. */
    while (!(I2C_SR1(dev->device) & I2C_SR1_ADDR)) {
        if ( diff_ticks(t0,systicks()) > dev->timeout ) {
            log_message(ERROR, I2C_BUS,"I2C ADDRESS ACK TIMEOUT!");
            return I2C_Addr_Timeout;
        }
    }

    /* program ACK = 0 for reading a single byte */
    if (n == 1)
        i2c_disable_ack(dev->device);

    /* Clearing ADDR condition sequence. */
    (void)(I2C_SR1(dev->device));
    (void)I2C_SR2(dev->device);

    /* program ACK = 0 for a two byte reading */
    if (n == 2) {
        while (!(I2C_SR1(dev->device) & I2C_SR1_RxNE)) {
            if ( diff_ticks(t0,systicks()) > dev->timeout ) {
                log_message(ERROR, I2C_BUS,"I2C RxNE TIMEOUT!");
                return I2C_Read_Timeout;
            }
        }
        i2c_disable_ack(dev->device);
    }
    i2c_send_stop(dev->device);


    for (size_t i = 0; i < n; ++i) {
        while (!(I2C_SR1(dev->device) & I2C_SR1_RxNE)) {
            if ( diff_ticks(t0,systicks()) > dev->timeout ) {
                log_message(ERROR, I2C_BUS,"I2C RxNE TIMEOUT!");
                return I2C_Read_Timeout;
            }
        }

         // TODO: change the condition to a specific byte count.
        if ((I2C_SR1(dev->device) & I2C_SR1_BTF) && n > 2) {
            i2c_disable_ack(dev->device);
            res[i++] = i2c_get_data(dev->device);
            i2c_send_stop(dev->device);
        }
        res[i] = i2c_get_data(dev->device);
    }

    return I2C_Ok;
}

static I2C_Fails_t NO_OPT i2c_transfer(I2C_Control_t *dev, const uint8_t *w, 
                                        size_t wn, uint8_t *r, size_t rn) {
    I2C_Fails_t status = I2C_Ok;

    if (wn) {
        status = i2c_write(dev, w, wn);
        if (status != I2C_Ok) {
            log_message(ERROR, I2C_BUS, i2c_fail_to_string(status));
            return status;
        }
    }

    if (rn) {
        status = i2c_read(dev, r, rn);
        if (status != I2C_Ok) {
            log_message(ERROR, I2C_BUS, i2c_fail_to_string(status));
            return status;
        }
    } else {
        i2c_send_stop(dev->device);
    }

    return I2C_Ok;
}

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

bool i2c_is_busy(I2C_Control_t *dev) {
    return dev->state != I2C_STATE_IDLE;
}

I2C_State_t i2c_get_state(I2C_Control_t *dev) {
    return dev->state;
}

I2C_Fails_t i2c_get_error(I2C_Control_t *dev) {
    return dev->error;
}

void i2c_abort(I2C_Control_t *dev) {
    /* Disable DMA requests */
    I2C_CR2(dev->device) &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
    
    /* Disable I2C interrupts */
    I2C_CR2(dev->device) &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    
    /* Send STOP condition */
    i2c_send_stop(dev->device);
    
    /* Reset state */
    dev->state = I2C_STATE_IDLE;
    dev->error = I2C_Ok;
}

/* ==========================================================================
 * Interrupt-Based Implementation
 * ========================================================================== */

void i2c_init_it(I2C_Control_t *dev, uint8_t priority) {
    /* Initialize async state */
    dev->state = I2C_STATE_IDLE;
    dev->mode = I2C_MODE_IT;
    dev->error = I2C_Ok;
    dev->callback = NULL;
    
    /* Configure NVIC for the board's I2C peripheral */
    if (dev->device == BOARD_I2C) {
        nvic_set_priority(BOARD_I2C_EV_IRQ, priority << 4);
        nvic_set_priority(BOARD_I2C_ER_IRQ, priority << 4);
        nvic_enable_irq(BOARD_I2C_EV_IRQ);
        nvic_enable_irq(BOARD_I2C_ER_IRQ);
    }
    
    log_message(INFO, I2C_BUS, "I2C interrupt mode initialized");
}

I2C_Fails_t i2c_write_it(I2C_Control_t *dev, const uint8_t *data, size_t len,
                         I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    /* Check if bus is busy */
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Setup transfer */
    dev->tx_buf = data;
    dev->tx_len = len;
    dev->tx_count = 0;
    dev->rx_buf = NULL;
    dev->rx_len = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_TX;
    dev->direction = I2C_DIR_WRITE;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_IT;
    
    /* Enable I2C interrupts */
    I2C_CR2(dev->device) |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN;
    
    /* Generate START condition */
    i2c_send_start(dev->device);
    
    return I2C_Ok;
}

I2C_Fails_t i2c_read_it(I2C_Control_t *dev, uint8_t *data, size_t len,
                        I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Setup transfer */
    dev->tx_buf = NULL;
    dev->tx_len = 0;
    dev->rx_buf = data;
    dev->rx_len = len;
    dev->rx_count = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_RX;
    dev->direction = I2C_DIR_READ;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_IT;
    
    /* Enable ACK for multi-byte reads */
    if (len > 1) {
        i2c_enable_ack(dev->device);
    }
    
    /* Enable I2C interrupts */
    I2C_CR2(dev->device) |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN;
    
    /* Generate START condition */
    i2c_send_start(dev->device);
    
    return I2C_Ok;
}

I2C_Fails_t i2c_read_reg_it(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data,
                            size_t len, I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Store register address */
    i2c_reg_addr_buf = regAddr;
    
    /* Setup transfer: write reg addr, then read data */
    dev->tx_buf = &i2c_reg_addr_buf;
    dev->tx_len = 1;
    dev->tx_count = 0;
    dev->rx_buf = data;
    dev->rx_len = len;
    dev->rx_count = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_TX_RX;
    dev->direction = I2C_DIR_WRITE;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_IT;
    
    /* Enable I2C interrupts */
    I2C_CR2(dev->device) |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN;
    
    /* Generate START condition */
    i2c_send_start(dev->device);
    
    return I2C_Ok;
}

I2C_Fails_t i2c_write_reg_it(I2C_Control_t *dev, uint8_t regAddr,
                             const uint8_t *data, size_t len,
                             I2C_Callback_t callback) {
    /* For register write, we need to send regAddr + data
     * This is a limitation - caller should prepare buffer with reg addr prepended
     * Or we could use a scatter-gather approach */
    (void)regAddr;
    return i2c_write_it(dev, data, len, callback);
}

/* ==========================================================================
 * DMA-Based Implementation
 * ========================================================================== */

#if I2C_DMA_ENABLED

void i2c_init_dma(I2C_Control_t *dev, uint8_t priority) {
    /* Initialize async state */
    dev->state = I2C_STATE_IDLE;
    dev->mode = I2C_MODE_DMA;
    dev->error = I2C_Ok;
    dev->callback = NULL;
    
    rcc_periph_clock_enable(BOARD_I2C_DMA_RCC);
    
    if (dev->device == BOARD_I2C) {
        /* Configure NVIC for DMA channels */
        nvic_set_priority(BOARD_I2C_DMA_TX_IRQ, priority << 4);
        nvic_set_priority(BOARD_I2C_DMA_RX_IRQ, priority << 4);
        nvic_enable_irq(BOARD_I2C_DMA_TX_IRQ);
        nvic_enable_irq(BOARD_I2C_DMA_RX_IRQ);
        
        /* Also enable I2C error interrupt */
        nvic_set_priority(BOARD_I2C_ER_IRQ, priority << 4);
        nvic_enable_irq(BOARD_I2C_ER_IRQ);
    }
    
    log_message(INFO, I2C_BUS, "I2C DMA mode initialized");
}

/**
 * @brief Undo a DMA transfer whose polled start phase failed
 *
 * Releases the bus, disarms the DMA stream and returns the driver to idle,
 * so the next transfer starts from a clean state.
 */
static I2C_Fails_t i2c_dma_start_failed(I2C_Control_t *dev, uint8_t stream,
                                        I2C_Fails_t err) {
    i2c_send_stop(dev->device);
    i2c_dma_disable(stream);
    I2C_CR2(dev->device) &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
    dev->state = I2C_STATE_IDLE;
    dev->error = err;
    return err;
}

static void i2c_dma_tx_setup(I2C_Control_t *dev, const uint8_t *data, size_t len) {
    uint32_t dma = BOARD_I2C_DMA;
    uint8_t channel = BOARD_I2C_DMA_TX;
    
    /* Disable channel first */
    i2c_dma_disable(channel);
    
    /* Configure DMA channel */
    dma_set_peripheral_address(dma, channel, (uint32_t)&I2C_DR(dev->device));
    dma_set_memory_address(dma, channel, (uint32_t)data);
    dma_set_number_of_data(dma, channel, len);
    
#if defined(STM32F1)
    dma_set_read_from_memory(dma, channel);
#else
    dma_channel_select(dma, channel, BOARD_I2C_DMA_CHANNEL);
    dma_set_transfer_mode(dma, channel, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
#endif // defined(STM32F1)
    dma_enable_memory_increment_mode(dma, channel);
    dma_disable_peripheral_increment_mode(dma, channel);
    dma_set_peripheral_size(dma, channel, I2C_DMA_PSIZE_8BIT);
    dma_set_memory_size(dma, channel, I2C_DMA_MSIZE_8BIT);
    dma_set_priority(dma, channel, I2C_DMA_PL_HIGH);
    
    /* Enable transfer complete interrupt */
    dma_enable_transfer_complete_interrupt(dma, channel);
    
    /* Enable channel */
    i2c_dma_enable(channel);
}

static void i2c_dma_rx_setup(I2C_Control_t *dev, uint8_t *data, size_t len) {
    uint32_t dma = BOARD_I2C_DMA;
    uint8_t channel = BOARD_I2C_DMA_RX;
    
    /* Disable channel first */
    i2c_dma_disable(channel);
    
    /* Configure DMA channel */
    dma_set_peripheral_address(dma, channel, (uint32_t)&I2C_DR(dev->device));
    dma_set_memory_address(dma, channel, (uint32_t)data);
    dma_set_number_of_data(dma, channel, len);
    
#if defined(STM32F1)
    dma_set_read_from_peripheral(dma, channel);
#else
    dma_channel_select(dma, channel, BOARD_I2C_DMA_CHANNEL);
    dma_set_transfer_mode(dma, channel, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
#endif // defined(STM32F1)
    dma_enable_memory_increment_mode(dma, channel);
    dma_disable_peripheral_increment_mode(dma, channel);
    dma_set_peripheral_size(dma, channel, I2C_DMA_PSIZE_8BIT);
    dma_set_memory_size(dma, channel, I2C_DMA_MSIZE_8BIT);
    dma_set_priority(dma, channel, I2C_DMA_PL_HIGH);
    
    /* Enable transfer complete interrupt */
    dma_enable_transfer_complete_interrupt(dma, channel);
    
    /* Enable channel */
    i2c_dma_enable(channel);
}

I2C_Fails_t i2c_write_dma(I2C_Control_t *dev, const uint8_t *data, size_t len,
                          I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Setup transfer state */
    dev->tx_buf = data;
    dev->tx_len = len;
    dev->tx_count = 0;
    dev->rx_buf = NULL;
    dev->rx_len = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_TX;
    dev->direction = I2C_DIR_WRITE;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_DMA;
    
    /* Setup DMA TX */
    i2c_dma_tx_setup(dev, data, len);
    
    /* Enable DMA request and error interrupt */
    I2C_CR2(dev->device) |= I2C_CR2_DMAEN | I2C_CR2_ITERREN;
    
    /* Generate START and send address */
    i2c_send_start(dev->device);
    
    /* Wait for START, send address, wait for ACK */
    TickType_t t0 = systicks();
    I2C_Fails_t status = i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout);

    if (status == I2C_Ok) {
        i2c_send_7bit_address(dev->device, dev->addr, I2C_WRITE);
        status = i2c_wait_sr1(dev, I2C_SR1_ADDR, t0, I2C_Addr_Timeout);
    }
    if (status != I2C_Ok) {
        return i2c_dma_start_failed(dev, BOARD_I2C_DMA_TX, status);
    }
    
    /* Clear ADDR by reading SR1 and SR2 */
    (void)I2C_SR1(dev->device);
    (void)I2C_SR2(dev->device);
    
    /* DMA will now handle the transfer */
    return I2C_Ok;
}

I2C_Fails_t i2c_read_dma(I2C_Control_t *dev, uint8_t *data, size_t len,
                         I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Setup transfer state */
    dev->tx_buf = NULL;
    dev->tx_len = 0;
    dev->rx_buf = data;
    dev->rx_len = len;
    dev->rx_count = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_RX;
    dev->direction = I2C_DIR_READ;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_DMA;
    
    /* Setup DMA RX */
    i2c_dma_rx_setup(dev, data, len);
    
    /* For DMA reads > 1 byte, enable LAST bit for automatic NACK on last byte */
    if (len > 1) {
        I2C_CR2(dev->device) |= I2C_CR2_LAST;
        i2c_enable_ack(dev->device);
    } else {
        i2c_disable_ack(dev->device);
    }
    
    /* Enable DMA request and error interrupt */
    I2C_CR2(dev->device) |= I2C_CR2_DMAEN | I2C_CR2_ITERREN;
    
    /* Generate START */
    i2c_send_start(dev->device);
    
    /* Wait for START, send address with READ bit, wait for ACK */
    TickType_t t0 = systicks();
    I2C_Fails_t status = i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout);

    if (status == I2C_Ok) {
        i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);
        status = i2c_wait_sr1(dev, I2C_SR1_ADDR, t0, I2C_Addr_Timeout);
    }
    if (status != I2C_Ok) {
        return i2c_dma_start_failed(dev, BOARD_I2C_DMA_RX, status);
    }
    
    /* Clear ADDR */
    (void)I2C_SR1(dev->device);
    (void)I2C_SR2(dev->device);
    
    /* DMA will now handle the transfer */
    return I2C_Ok;
}

I2C_Fails_t i2c_read_reg_dma(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data,
                             size_t len, I2C_Callback_t callback) {
    if (dev->state != I2C_STATE_IDLE) {
        return I2C_Busy;
    }
    
    if (I2C_SR2(dev->device) & I2C_SR2_BUSY) {
        return I2C_Busy_Timeout;
    }
    
    /* Setup transfer state */
    dev->rx_buf = data;
    dev->rx_len = len;
    dev->rx_count = 0;
    dev->callback = callback;
    dev->state = I2C_STATE_BUSY_RX;
    dev->direction = I2C_DIR_READ;
    dev->error = I2C_Ok;
    dev->mode = I2C_MODE_DMA;
    
    /* Setup DMA RX before starting I2C transaction */
    i2c_dma_rx_setup(dev, data, len);
    
    /* Phase 1: Send register address (polling) */
    TickType_t t0 = systicks();
    i2c_send_start(dev->device);
    I2C_Fails_t status = i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout);

    if (status == I2C_Ok) {
        i2c_send_7bit_address(dev->device, dev->addr, I2C_WRITE);
        status = i2c_wait_sr1(dev, I2C_SR1_ADDR, t0, I2C_Addr_Timeout);
    }
    if (status != I2C_Ok) {
        return i2c_dma_start_failed(dev, BOARD_I2C_DMA_RX, status);
    }
    (void)I2C_SR1(dev->device);
    (void)I2C_SR2(dev->device);

    /* Send register address */
    i2c_send_data(dev->device, regAddr);
    status = i2c_wait_sr1(dev, I2C_SR1_BTF, t0, I2C_Write_Timeout);
    if (status != I2C_Ok) {
        return i2c_dma_start_failed(dev, BOARD_I2C_DMA_RX, status);
    }
    
    /* Phase 2: Repeated START for read with DMA */
    
    /* For DMA reads > 1 byte, enable LAST bit for automatic NACK on last byte */
    if (len > 1) {
        I2C_CR2(dev->device) |= I2C_CR2_LAST;
        i2c_enable_ack(dev->device);
    } else {
        i2c_disable_ack(dev->device);
    }
    
    /* Enable DMA request and error interrupt */
    I2C_CR2(dev->device) |= I2C_CR2_DMAEN | I2C_CR2_ITERREN;
    
    /* Generate repeated START */
    i2c_send_start(dev->device);
    
    /* Wait for START, send address with READ bit, wait for ACK */
    status = i2c_wait_sr1(dev, I2C_SR1_SB, t0, I2C_Busy_Timeout);

    if (status == I2C_Ok) {
        i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);
        status = i2c_wait_sr1(dev, I2C_SR1_ADDR, t0, I2C_Addr_Timeout);
    }
    if (status != I2C_Ok) {
        return i2c_dma_start_failed(dev, BOARD_I2C_DMA_RX, status);
    }
    
    /* Clear ADDR - this starts the DMA transfer */
    (void)I2C_SR1(dev->device);
    (void)I2C_SR2(dev->device);
    
    /* DMA will now handle the transfer and call the callback when done */
    return I2C_Ok;
}

/* ==========================================================================
 * Interrupt Service Routines
 * ========================================================================== */

void i2c_ev_isr(I2C_Control_t *dev) {
    uint32_t sr1 = I2C_SR1(dev->device);
    uint32_t sr2 = I2C_SR2(dev->device);
    
    /* Start bit sent */
    if (sr1 & I2C_SR1_SB) {
        if (dev->direction == I2C_DIR_WRITE) {
            i2c_send_7bit_address(dev->device, dev->addr, I2C_WRITE);
        } else {
            i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);
        }
        return;
    }
    
    /* Address sent, ACK received */
    if (sr1 & I2C_SR1_ADDR) {
        /* Clear ADDR by reading SR1 and SR2 */
        (void)sr1;
        (void)sr2;
        
        if (dev->direction == I2C_DIR_READ) {
            if (dev->rx_len == 1) {
                /* Single byte read: disable ACK before clearing ADDR */
                i2c_disable_ack(dev->device);
                i2c_send_stop(dev->device);
            } else if (dev->rx_len == 2) {
                /* Two byte read: set POS and disable ACK */
                I2C_CR1(dev->device) |= I2C_CR1_POS;
                i2c_disable_ack(dev->device);
            }
        }
        return;
    }
    
    /* TX buffer empty - ready to send next byte */
    if ((sr1 & I2C_SR1_TxE) && dev->direction == I2C_DIR_WRITE) {
        if (dev->tx_count < dev->tx_len) {
            i2c_send_data(dev->device, dev->tx_buf[dev->tx_count++]);
        } else {
            /* TX complete */
            if (dev->state == I2C_STATE_BUSY_TX_RX && dev->rx_len > 0) {
                /* Switch to RX phase with repeated start */
                dev->direction = I2C_DIR_READ;
                dev->rx_count = 0;
                
                if (dev->rx_len > 1) {
                    i2c_enable_ack(dev->device);
                }
                
                i2c_send_start(dev->device);
            } else {
                /* All done, send STOP */
                I2C_CR2(dev->device) &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
                i2c_send_stop(dev->device);
                
                dev->state = I2C_STATE_IDLE;
                if (dev->callback) {
                    dev->callback(dev, I2C_Ok);
                }
            }
        }
        return;
    }
    
    /* RX buffer not empty - data received */
    if ((sr1 & I2C_SR1_RxNE) && dev->direction == I2C_DIR_READ) {
        if (dev->rx_count < dev->rx_len) {
            dev->rx_buf[dev->rx_count++] = i2c_get_data(dev->device);
            
            /* Handle end of reception */
            if (dev->rx_count == dev->rx_len - 1 && dev->rx_len > 2) {
                /* Next byte is last: disable ACK */
                i2c_disable_ack(dev->device);
                i2c_send_stop(dev->device);
            } else if (dev->rx_count == dev->rx_len) {
                /* All bytes received */
                I2C_CR2(dev->device) &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
                
                dev->state = I2C_STATE_IDLE;
                if (dev->callback) {
                    dev->callback(dev, I2C_Ok);
                }
            }
        }
        return;
    }
    
    /* Byte transfer finished (for 2-byte reads) */
    if ((sr1 & I2C_SR1_BTF) && dev->direction == I2C_DIR_READ && dev->rx_len == 2) {
        i2c_send_stop(dev->device);
        dev->rx_buf[0] = i2c_get_data(dev->device);
        dev->rx_buf[1] = i2c_get_data(dev->device);
        dev->rx_count = 2;
        
        I2C_CR2(dev->device) &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN);
        I2C_CR1(dev->device) &= ~I2C_CR1_POS;
        
        dev->state = I2C_STATE_IDLE;
        if (dev->callback) {
            dev->callback(dev, I2C_Ok);
        }
    }
}

void i2c_er_isr(I2C_Control_t *dev) {
    uint32_t sr1 = I2C_SR1(dev->device);
    I2C_Fails_t error = I2C_Ok;
    
    /* Bus error */
    if (sr1 & I2C_SR1_BERR) {
        I2C_SR1(dev->device) &= ~I2C_SR1_BERR;
        error = I2C_Bus_Error;
    }
    
    /* Arbitration lost */
    if (sr1 & I2C_SR1_ARLO) {
        I2C_SR1(dev->device) &= ~I2C_SR1_ARLO;
        error = I2C_Arbitration_Lost;
    }
    
    /* Acknowledge failure */
    if (sr1 & I2C_SR1_AF) {
        I2C_SR1(dev->device) &= ~I2C_SR1_AF;
        i2c_send_stop(dev->device);
        error = I2C_Nack;
    }
    
    /* Overrun/Underrun */
    if (sr1 & I2C_SR1_OVR) {
        I2C_SR1(dev->device) &= ~I2C_SR1_OVR;
        error = I2C_Overrun;
    }
    
    if (error != I2C_Ok) {
        /* Disable interrupts */
        I2C_CR2(dev->device) &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        I2C_CR2(dev->device) &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        
        dev->error = error;
        dev->state = I2C_STATE_ERROR;
        
        if (dev->callback) {
            dev->callback(dev, error);
        }
        
        dev->state = I2C_STATE_IDLE;
    }
}

void i2c_dma_tx_isr(I2C_Control_t *dev) {
    /* Clear DMA transfer complete flag */
    if (dma_get_interrupt_flag(BOARD_I2C_DMA, BOARD_I2C_DMA_TX, DMA_TCIF)) {
        dma_clear_interrupt_flags(BOARD_I2C_DMA, BOARD_I2C_DMA_TX, DMA_TCIF);
        
        /* Disable DMA channel */
        i2c_dma_disable(BOARD_I2C_DMA_TX);
        
        /* Wait for BTF (byte transfer finished) before STOP */
        /* Bounded: this runs in an ISR; BTF normally follows within one byte time */
        for (uint32_t spins = 0;
             spins < I2C_ISR_BTF_SPINS && !(I2C_SR1(dev->device) & I2C_SR1_BTF);
             spins++);
        
        /* Send STOP condition */
        i2c_send_stop(dev->device);
        
        /* Disable DMA request */
        I2C_CR2(dev->device) &= ~I2C_CR2_DMAEN;
        
        dev->tx_count = dev->tx_len;
        dev->state = I2C_STATE_IDLE;
        
        if (dev->callback) {
            dev->callback(dev, I2C_Ok);
        }
    }
    
    /* Check for DMA error */
    if (dma_get_interrupt_flag(BOARD_I2C_DMA, BOARD_I2C_DMA_TX, DMA_TEIF)) {
        dma_clear_interrupt_flags(BOARD_I2C_DMA, BOARD_I2C_DMA_TX, DMA_TEIF);
        i2c_dma_disable(BOARD_I2C_DMA_TX);
        
        i2c_send_stop(dev->device);
        I2C_CR2(dev->device) &= ~I2C_CR2_DMAEN;
        
        dev->error = I2C_DMA_Error;
        dev->state = I2C_STATE_IDLE;
        
        if (dev->callback) {
            dev->callback(dev, I2C_DMA_Error);
        }
    }
}

void i2c_dma_rx_isr(I2C_Control_t *dev) {
    /* Clear DMA transfer complete flag */
    if (dma_get_interrupt_flag(BOARD_I2C_DMA, BOARD_I2C_DMA_RX, DMA_TCIF)) {
        dma_clear_interrupt_flags(BOARD_I2C_DMA, BOARD_I2C_DMA_RX, DMA_TCIF);
        
        /* Disable DMA channel */
        i2c_dma_disable(BOARD_I2C_DMA_RX);
        
        /* Send STOP condition */
        i2c_send_stop(dev->device);
        
        /* Disable DMA request and LAST bit */
        I2C_CR2(dev->device) &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        
        dev->rx_count = dev->rx_len;
        dev->state = I2C_STATE_IDLE;
        
        if (dev->callback) {
            dev->callback(dev, I2C_Ok);
        }
    }
    
    /* Check for DMA error */
    if (dma_get_interrupt_flag(BOARD_I2C_DMA, BOARD_I2C_DMA_RX, DMA_TEIF)) {
        dma_clear_interrupt_flags(BOARD_I2C_DMA, BOARD_I2C_DMA_RX, DMA_TEIF);
        i2c_dma_disable(BOARD_I2C_DMA_RX);
        
        i2c_send_stop(dev->device);
        I2C_CR2(dev->device) &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        
        dev->error = I2C_DMA_Error;
        dev->state = I2C_STATE_IDLE;
        
        if (dev->callback) {
            dev->callback(dev, I2C_DMA_Error);
        }
    }
}

#endif /* I2C_DMA_ENABLED */

// i2c.c
