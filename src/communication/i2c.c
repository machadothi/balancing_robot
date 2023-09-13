/* A stm32f103 application I2C library
 * Warren W. Gay VE3WWG
 * Sat Nov 25 11:56:51 2017
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

static const char *i2c_msg[] = {
    "OK",
    "Address timeout",
    "Address NAK",
    "Write timeout",
    "Read timeout"
};

jmp_buf i2c_exception;

/*********************************************************************
 * Compute the difference in ticks:
 *********************************************************************/

static inline TickType_t
diff_ticks(TickType_t early,TickType_t later) {

    if ( later >= early )
        return later - early;
    return ~(TickType_t)0 - early + 1 + later;
}

/*********************************************************************
 * Return a character string message for I2C_Fails code
 *********************************************************************/

const char *
i2c_error(I2C_Fails fcode) {
    int icode = (int)fcode;

    if ( icode < 0 || icode > (int)I2C_Read_Timeout )
        return "Bad I2C_Fails code";
    return i2c_msg[icode];
}

/*********************************************************************
 * Configure I2C device for 100 kHz, 7-bit addresses
 *********************************************************************/

void
i2c_configure(I2C_Control *dev,uint32_t i2c, uint8_t address,uint32_t ticks) {

    dev->device = i2c;
    dev->addr = (address|((0)&7));
    dev->timeout = ticks;

    i2c_peripheral_disable(dev->device);
    // i2c_reset(dev->device);
    I2C_CR1(dev->device) &= ~I2C_CR1_STOP;    // Clear stop
    i2c_set_standard_mode(dev->device);    // 100 kHz mode
    i2c_set_clock_frequency(dev->device,I2C_CR2_FREQ_36MHZ); // APB Freq
    i2c_set_trise(dev->device,36);        // 1000 ns
    i2c_set_dutycycle(dev->device,I2C_CCR_DUTY_DIV2);
    i2c_set_ccr(dev->device,180);        // 100 kHz <= 180 * 1 /36M
    i2c_set_own_7bit_slave_address(dev->device,0x23); // Necessary?
    i2c_peripheral_enable(dev->device);
}

/*********************************************************************
 * Return when I2C is not busy
 *********************************************************************/

bool
i2c_is_busy(uint32_t i2c) {
    return (I2C_SR2(i2c) & I2C_SR2_BUSY);
}

/*********************************************************************
 * Return when I2C slave set ACK fails
 *********************************************************************/

bool
i2c_slv_ack_fail(uint32_t i2c) {
    return ( I2C_SR1(i2c) & I2C_SR1_AF );
}

/*********************************************************************
 * Clears I2C Status Registers 1 and 2
 *********************************************************************/

void
i2c_clear_status_flags(uint32_t i2c, Status_Flag flag) {
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

/*********************************************************************
 * Clears I2C Status Registers 1 and 2
 *********************************************************************/

bool
i2c_slave_found(uint32_t i2c) {
    return (I2C_SR1(i2c) & I2C_SR1_ADDR);
}

/*********************************************************************
 * Return if the Start bit was successfully set.
 *********************************************************************/

bool
i2c_start_bit(uint32_t i2c){
    return (I2C_SR1(i2c) & I2C_SR1_SB);
}

/*********************************************************************
 * Start I2C Read/Write Transaction with indicated 7-bit address:
 *********************************************************************/

I2C_Fails
i2c_start_addr(I2C_Control *dev, enum I2C_RW rw) {
    i2c_clear_status_flags(dev->device, SF_2);
    i2c_clear_stop(dev->device);        // Do not generate a Stop
    if ( rw == Read )
        i2c_enable_ack(dev->device);

    i2c_send_start(dev->device);        // Generate a Start/Restart

	// Loop until ready:
    TickType_t t0 = systicks();
    while ( !i2c_start_bit(dev->device) && i2c_is_busy(dev->device)) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Busy_Timeout;
		// taskYIELD(); //?
	}

    // Send Address & R/W flag:
    i2c_send_7bit_address(dev->device,dev->addr, rw == Read ? I2C_READ : I2C_WRITE);

    // Wait until completion, NAK or timeout
    t0 = systicks();
    while ( !i2c_slave_found(dev->device) ) {
        if ( i2c_slv_ack_fail(dev->device)) {
            i2c_send_stop(dev->device);
            i2c_clear_status_flags(dev->device, SF_BOTH);
            return I2C_Addr_NAK; 
        }
        if ( diff_ticks(t0,systicks()) > dev->timeout )
            return I2C_Addr_Timeout;        
    }

    i2c_clear_status_flags(dev->device, SF_2);
    return I2C_Ok;
}

bool i2c_byte_transfer_finished(uint32_t i2c) {
    return (I2C_SR1(i2c) & (I2C_SR1_BTF));
}

/*********************************************************************
 * Write one byte of data, then initiate a repeated start for a
 * read to follow.
 *********************************************************************/

I2C_Fails
i2c_write_restart(I2C_Control *dev,uint8_t byte) {
	taskENTER_CRITICAL();
	i2c_send_data(dev->device,byte);
	// Must set start before byte has written out
	i2c_send_start(dev->device);
	taskEXIT_CRITICAL();

	// Wait for transmit to complete
	TickType_t t0 = systicks();
	while ( !i2c_byte_transfer_finished(dev->device) ) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Write_Timeout;
	}

	// Loop until restart ready:
	t0 = systicks();
    while ( !i2c_start_bit(dev->device) && i2c_is_busy(dev->device)) {
		if ( diff_ticks(t0,systicks()) > dev->timeout )
			return I2C_Busy_Timeout;
	}

	// Send Address & Read command bit
	i2c_send_7bit_address(dev->device, dev->addr, I2C_READ);

	// Wait until completion, NAK or timeout
	t0 = systicks();
    while ( !i2c_slave_found(dev->device) ) {
        if ( i2c_slv_ack_fail(dev->device)) {
            i2c_send_stop(dev->device);
            i2c_clear_status_flags(dev->device, SF_BOTH);
            return I2C_Addr_NAK; 
        }
        if ( diff_ticks(t0,systicks()) > dev->timeout )
            return I2C_Addr_Timeout;        
    }

	i2c_clear_status_flags(dev->device, SF_2);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------
// FIXXXXXXXXX
I2C_Fails
readBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    readByte(dev, regAddr, &b);
    *data = b & (1 << bitNum);
    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
readByte(I2C_Control *dev, uint8_t regAddr, uint8_t *data) {

    i2c_start_addr(dev,Write);
    i2c_write_restart(dev, regAddr);

    // i2c_start_addr(dev,Read);
    *data = i2c_get_data(dev->device);;
    i2c_stop(dev);

    return I2C_Ok;
}

// -----------------------------------------------------------------------------

I2C_Fails
writeBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b = 0;
    readByte(dev, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(dev, regAddr, b);
}

// -----------------------------------------------------------------------------

I2C_Fails
writeBits(I2C_Control *dev, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
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

I2C_Fails
writeByte(I2C_Control *dev, uint8_t regAddr, uint8_t data) {
    i2c_start_addr(dev,Write);
    i2c_send_data(dev->device,regAddr);

    TickType_t t0 = systicks();
    while ( i2c_slv_ack_fail(dev->device) ) {
        if ( diff_ticks(t0,systicks()) > dev->timeout )
            return I2C_Addr_NAK;        
    }

    i2c_send_data(dev->device, data);
    t0 = systicks();
    while ( i2c_slv_ack_fail(dev->device) ) {
        if ( diff_ticks(t0,systicks()) > dev->timeout )
            return I2C_Addr_NAK;        
    }

    i2c_stop(dev);

    return I2C_Ok;
}

// i2c.c
