/**
 * @file i2c.c
 * @brief I2C driver for STM32F103
 * 
 * Features:
 *   - Master mode only
 *   - Polling (no interrupts)
 *   - Bus recovery for stuck slaves
 *   - Configurable timeout
 * 
 * Hardware:
 *   - I2C1: PB6=SCL, PB7=SDA
 *   - 100 kHz standard mode
 * 
 * @author Thiago Cunha
 * @date 2023
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include <FreeRTOS.h>
#include <task.h>

#include "i2c.h"
#include "log/log.h"

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define I2C_SCL_PIN     GPIO6
#define I2C_SDA_PIN     GPIO7
#define I2C_PORT        GPIOB
#define I2C_PERIPH      I2C1

#define I2C_APB_FREQ    36      /* APB1 frequency in MHz */
#define I2C_TRISE_VAL   0x25    /* Rise time for 100kHz */
#define I2C_CCR_VAL     180     /* CCR for 100kHz: 180 * 1/36MHz */

#define NO_OPT __attribute__((optimize("O0")))
#define systicks    xTaskGetTickCount

/* ==========================================================================
 * Private Function Prototypes
 * ========================================================================== */

/**
 * @brief Converts I2C_Fails_t enum to string.
 * 
 * @param fail I2C_Fails_t enum value
 * @return const char* String representation of the I2C_Fails_t
 */
static const char* i2c_fail_to_string(I2C_Fails_t fail) {
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
        default:
            return "UNKNOWN_ERROR";
    }
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
void i2c_bus_recovery(void) {
    /* Configure SCL as GPIO output */
    gpio_set_mode(I2C_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_OPENDRAIN, I2C_SCL_PIN);
    
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
    gpio_set_mode(I2C_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, I2C_SCL_PIN);
}

void NO_OPT i2c_setup_peripheral(void) {
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    
    /* Configure I2C pins as open-drain alternate function */
    gpio_set_mode(I2C_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  I2C_SCL_PIN | I2C_SDA_PIN);
    
    /* Set idle high */
    gpio_set(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
    
    /* Recover bus if stuck */
    i2c_bus_recovery();
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
    gpio_set_mode(I2C_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_OPENDRAIN, I2C_SCL_PIN | I2C_SDA_PIN);
    
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
    gpio_set_mode(I2C_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, I2C_SCL_PIN | I2C_SDA_PIN);
    
    /* Step 6: Software reset via SWRST bit */
    I2C_CR1(dev->device) |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 100; i++);
    I2C_CR1(dev->device) &= ~I2C_CR1_SWRST;
    
    /* Configure I2C peripheral */
    i2c_set_standard_mode(dev->device);
    i2c_set_clock_frequency(dev->device, I2C_APB_FREQ);
    i2c_set_trise(dev->device, I2C_TRISE_VAL);
    i2c_set_dutycycle(dev->device, I2C_CCR_DUTY_DIV2);
    i2c_set_ccr(dev->device, I2C_CCR_VAL);
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
    while ( !( (I2C_SR1(dev->device) & I2C_SR1_SB)
        && (I2C_SR2(dev->device) & I2C_SR2_MSL)
        && (I2C_SR2(dev->device) & I2C_SR2_BUSY) ));

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

    while (!(I2C_SR1(dev->device) & I2C_SR1_SB));

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

// i2c.c
