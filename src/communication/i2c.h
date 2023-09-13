/* A stm32f103 library for user applications
 * Warren W. Gay VE3WWG
 * Sat Nov 25 11:53:53 2017
 */
#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <setjmp.h>

#include <libopencm3/stm32/i2c.h>

typedef enum {
    I2C_Ok = 0,
    I2C_Addr_Timeout,
    I2C_Addr_NAK,
    I2C_Write_Timeout,
    I2C_Read_Timeout,
    I2C_Busy_Timeout
} I2C_Fails;

enum I2C_RW {
    Read = 1,
    Write = 0
};

typedef enum {
    SF_1,
    SF_2,
    SF_BOTH
} Status_Flag;

typedef struct {
    uint32_t    device;        // I2C device
    uint8_t addr;
    uint32_t    timeout;    // Ticks
} I2C_Control;

extern jmp_buf i2c_exception;

const char *i2c_error(I2C_Fails fcode);

void i2c_configure(I2C_Control *dev,uint32_t i2c, uint8_t address, uint32_t ticks);
bool i2c_is_busy(uint32_t i2c);
bool i2c_slv_ack_fail(uint32_t i2c);
void i2c_clear_status_flags(uint32_t i2c, Status_Flag flag);
bool i2c_slave_found(uint32_t i2c);
bool i2c_start_bit(uint32_t i2c);
I2C_Fails i2c_start_addr(I2C_Control *dev, enum I2C_RW rw);
I2C_Fails i2c_write_restart(I2C_Control *dev,uint8_t byte);
bool i2c_byte_transfer_finished(uint32_t i2c);

inline void i2c_stop(I2C_Control *dev) { i2c_send_stop(dev->device); }

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
I2C_Fails readBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
I2C_Fails readByte(I2C_Control *dev, uint8_t regAddr, uint8_t *data);

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
I2C_Fails writeBit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, 
                        uint8_t data);

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
I2C_Fails writeBits(I2C_Control *dev, uint8_t regAddr, uint8_t bitStart, 
                uint8_t length, uint8_t data);

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
I2C_Fails writeByte(I2C_Control *dev, uint8_t regAddr, uint8_t data);

#endif // I2C_H