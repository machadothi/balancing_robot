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

#define systicks    xTaskGetTickCount

#define NO_OPT __attribute__((optimize("O0")))

/**
 * Compute the difference in ticks:
*/
static inline TickType_t
diff_ticks(TickType_t early,TickType_t later) {

    if ( later >= early )
        return later - early;
    return ~(TickType_t)0 - early + 1 + later;
}

// -----------------------------------------------------------------------------

void
NO_OPT i2c_configure(I2C_Control *dev,uint32_t i2c, uint8_t address,uint32_t ticks) {

    dev->device = i2c;
    dev->addr = address;
    dev->timeout = ticks;

    i2c_reset(dev->device);
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

bool
NO_OPT i2c_is_busy(uint32_t i2c) {
    return ((I2C_SR2(i2c) & I2C_SR2_BUSY) == I2C_SR2_BUSY);
}

// -----------------------------------------------------------------------------

bool
NO_OPT i2c_slv_ack_fail(uint32_t i2c) {
    return (( I2C_SR1(i2c) & I2C_SR1_AF ) == I2C_SR1_AF );
}

// -----------------------------------------------------------------------------

void
NO_OPT i2c_clear_status_flags(uint32_t i2c, Status_Flag flag) {
    switch (flag)
    {
    case SF_1:
        (void)I2C_SR1(i2c);
        break;
    case SF_2:
        (void)I2C_SR2(i2c);
        break;
    case SF_BOTH:
        (void)I2C_SR1(i2c);
        (void)I2C_SR2(i2c);
        break;
    
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

bool
NO_OPT i2c_slave_found(uint32_t i2c) {
    return ((I2C_SR1(i2c) & I2C_SR1_ADDR) == I2C_SR1_ADDR );
}

// -----------------------------------------------------------------------------

bool
NO_OPT i2c_start_bit(uint32_t i2c) {
    return ((I2C_SR1(i2c) & I2C_SR1_SB) == I2C_SR1_SB );
}

void
NO_OPT i2c_reset(uint32_t i2c) {
    I2C_CR1(i2c) |= I2C_CR1_SWRST;
    I2C_CR1(i2c) &= ~I2C_CR1_SWRST;
}

// -----------------------------------------------------------------------------

I2C_Fails
i2c_start_addr(I2C_Control *dev, enum I2C_RW rw) {
    TickType_t t0 = systicks();

    i2c_send_start(dev->device);        // Generate a Start/Restart

	// Loop until ready:
    while ( !i2c_start_bit(dev->device) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Busy_Timeout;

		taskYIELD();
	}

    // Send Address & R/W flag:
    i2c_send_7bit_address(dev->device,dev->addr, 
      rw == Read ? I2C_READ : I2C_WRITE);

    // Wait until completion, NAK or timeout
    t0 = systicks();
    while ( !i2c_slave_found(dev->device) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Busy_Timeout;

		taskYIELD();
	}

    i2c_clear_status_flags(dev->device, SF_BOTH);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
readBytes(I2C_Control *dev, uint8_t regAddr, uint8_t *data, uint8_t length) {

    TickType_t t0 = systicks();
	// Loop until ready:
    while ( i2c_is_busy(dev->device) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Busy_Timeout;

		taskYIELD();
	}

    uint8_t count = 0;

    i2c_start_addr(dev,Write);
    i2c_write_restart(dev, regAddr);
    
    for (int k = 0; k < length; k ++) {
        data[count] = i2c_get_data(dev->device);
        count++;
    }

    i2c_send_stop(dev->device);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
NO_OPT readBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    readByte(dev, regAddr, &b);
    *data = b & (1 << bitNum);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
writeBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b = 0;
    readByte(dev, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(dev, regAddr, b);
}

// -----------------------------------------------------------------------------


bool NO_OPT
i2c_byte_transfer_finished(uint32_t i2c) {
    return ((I2C_SR1(i2c) & (I2C_SR1_BTF)) == I2C_SR1_BTF);
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
i2c_write_restart(I2C_Control *dev,uint8_t byte) {
	i2c_send_data(dev->device,byte);
	// Must set start before byte has written out
	i2c_send_start(dev->device);

	// Wait for transmit to complete
	while ( !i2c_byte_transfer_finished(dev->device) ) {
        taskYIELD();
	}

	// Loop until restart ready:
    while ( !i2c_start_bit(dev->device)) {
        taskYIELD();
	}

    i2c_clear_status_flags(dev->device, SF_1);
	// Send Address & Read command bit
	i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);

	// Wait until completion, NAK or timeout
    while ( !i2c_slave_found(dev->device) ) {        
        taskYIELD();
    }

	i2c_clear_status_flags(dev->device, SF_BOTH);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
writeBits(I2C_Control *dev, uint8_t regAddr, uint8_t bitStart, uint8_t length, 
  uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b = 0;
    readByte(dev, regAddr, &b);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return writeByte(dev, regAddr, b);
}

// -----------------------------------------------------------------------------

I2C_Fails NO_OPT
writeByte(I2C_Control *dev, uint8_t regAddr, uint8_t data) {
    
    uint8_t content[2] = {regAddr, data};

    i2c_transfer(dev->device, dev->addr, &content, 2, &data, 0);
    return I2C_Ok;
}
// -----------------------------------------------------------------------------

I2C_Fails
readByte(I2C_Control *dev, uint8_t regAddr, uint8_t *data) {
    taskENTER_CRITICAL();
    uint8_t d = 0;
    i2c_transfer(dev->device, dev->addr, &regAddr, 1, &d, 1);
    *data = d;
    taskEXIT_CRITICAL();
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

void NO_OPT
i2c_write_v1(uint32_t i2c, int addr, const uint8_t *data, size_t n)
{
	while ((I2C_SR2(i2c) & I2C_SR2_BUSY)) {
	}

	i2c_send_start(i2c);

	/* Wait for the end of the start condition, master mode selected, and BUSY bit set */
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

void NO_OPT
i2c_read_v1(uint32_t i2c, int addr, uint8_t *res, size_t n)
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
		*res = i2c_get_data(i2c);
	}

	return;
}

// -----------------------------------------------------------------------------

void NO_OPT
i2c_transfer(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn) {
	if (wn) {
		i2c_write_v1(i2c, addr, w, wn);
	}
	if (rn) {
		i2c_read_v1(i2c, addr, r, rn);
	} else {
		i2c_send_stop(i2c);
	}
}

// i2c.c
