/** A stm32f103 I2C library for user applications
 * Thiago Cunha
 * Thu Sep 14 2023
 *
 * Notes:
 *    1. Master I2C mode only
 *    2. No interrupts are used
 *    3. ReSTART I2C is not supported
 *    4. Uses PB6=SCL, PB7=SDA
 *    5. Requires GPIOB clock enabled
 *    6. PB6+PB7 must be GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN
 *    7. Requires rcc_periph_clock_enable(RCC_I2C1);
 *    8. Requires rcc_periph_clock_enable(RCC_AFIO);
 *    9. 100 kHz
 */
#include "FreeRTOS.h"
#include "task.h"

#include "i2c.h"

#define NO_OPT __attribute__((optimize("O0")))

/* Private functions */
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
static void i2c_transfer(uint32_t i2c, uint8_t addr, const uint8_t *w, 
  size_t wn, uint8_t *r, size_t rn);

/**
 * @brief Reads a given number of bytes from I2C bus. The STM32F1 has different
 * conditions for reading when n = 1, n = 2 and n > 2.
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param res destination buffer to read into
 * @param n number of bytes to read
 */
static void i2c_read(uint32_t i2c, int addr, uint8_t *res, size_t n);

/**
 * @brief writes data into I2C bus.
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param data content to write
 * @param n number of bytes to write 
 */
static void i2c_write(uint32_t i2c, int addr, const uint8_t *data, size_t n);

// -----------------------------------------------------------------------------

/* Public functions */

// TODO: Add parameters verification and error handling with I2C_Fails codes.

void NO_OPT
i2c_configure(I2C_Control *dev,uint32_t i2c, uint8_t address) {

    dev->device = i2c;
    dev->addr = address;

    i2c_peripheral_disable(dev->device);
    i2c_clear_stop(dev->device);
    i2c_set_standard_mode(dev->device);    // 100 kHz mode
    i2c_set_clock_frequency(dev->device, 36); // APB Freq
    i2c_set_trise(dev->device,0x5);        // 500 ns
    i2c_set_dutycycle(dev->device,I2C_CCR_DUTY_DIV2);
    i2c_set_ccr(dev->device,180);        // 100 kHz <= 180 * 1 /36M
    i2c_peripheral_enable(dev->device);
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
i2c_read_bit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    i2c_read_byte(dev, regAddr, &b);
    *data = b & (1 << bitNum);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
i2c_write_bit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b = 0;
    i2c_read_byte(dev, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return i2c_write_byte(dev, regAddr, b);
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
i2c_write_bits(I2C_Control *dev, uint8_t regAddr, uint8_t bitStart, uint8_t length, 
  uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    i2c_read_byte(dev, regAddr, &b);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return i2c_write_byte(dev, regAddr, b);
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
i2c_write_byte(I2C_Control *dev, uint8_t regAddr, uint8_t data) {
    
    uint8_t content[2] = {regAddr, data};

    i2c_transfer(dev->device, dev->addr, &content, 2, &data, 0);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
i2c_read_byte(I2C_Control *dev, uint8_t regAddr, uint8_t *data) {

    i2c_transfer(dev->device, dev->addr, &regAddr, 1, data, 1);

    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
i2c_read_bytes(I2C_Control *dev, uint8_t regAddr, uint8_t *data, 
  uint8_t length) {
    
    i2c_transfer(dev->device, dev->addr, &regAddr, 1, data, length);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

static void NO_OPT
i2c_write(uint32_t i2c, int addr, const uint8_t *data, size_t n)
{
    while ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
    }

    i2c_send_start(i2c);

    /* Wait for the end of the start condition, master mode selected, 
        and BUSY bit set */
    while ( !( (I2C_SR1(i2c) & I2C_SR1_SB)
        && (I2C_SR2(i2c) & I2C_SR2_MSL)
        && (I2C_SR2(i2c) & I2C_SR2_BUSY) ));

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);

    /* Waiting for address is transferred. */
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

    /* Clearing ADDR condition sequence. */
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < n; i++) {
        i2c_send_data(i2c, data[i]);
        while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
    }
}

// -----------------------------------------------------------------------------

static void NO_OPT
i2c_read(uint32_t i2c, int addr, uint8_t *res, size_t n)
{
    i2c_send_start(i2c);
    i2c_enable_ack(i2c);

    while (!(I2C_SR1(i2c) & I2C_SR1_SB));

    i2c_send_7bit_address(i2c, addr, I2C_READ);

    /* Waiting for address is transferred. */
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

    /* program ACK = 0 for reading a single byte */
    if (n == 1)
        i2c_disable_ack(i2c);

    /* Clearing ADDR condition sequence. */
    (void)(I2C_SR1(i2c));
    (void)I2C_SR2(i2c);

    /* program ACK = 0 for reading a two byte */
    if (n == 2) {
        i2c_disable_ack(i2c);
        while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
    }
    i2c_send_stop(i2c);

    while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));

    for (size_t i = 0; i < n; ++i) {
		if ((I2C_SR1(i2c) & I2C_SR1_BTF) && n > 2) {
			i2c_disable_ack(i2c);
            res[i++] = i2c_get_data(i2c);
            i2c_send_stop(i2c);
		}
		res[i] = i2c_get_data(i2c);
	}

    return;
}

// -----------------------------------------------------------------------------

static void NO_OPT
i2c_transfer(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, 
  uint8_t *r, size_t rn) {
    if (wn) {
        i2c_write(i2c, addr, w, wn);
    }
    if (rn) {
        i2c_read(i2c, addr, r, rn);
    } else {
        i2c_send_stop(i2c);
    }
}

// i2c.c
