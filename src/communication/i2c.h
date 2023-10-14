/* A stm32f103 I2C library for user applications
 * Thiago Cunha
 * Thu Sep 14 2023
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

typedef struct {
    uint32_t    device;        // I2C device
    uint8_t addr;             // Device address
} I2C_Control;

/**
 * @brief Configure I2C device for 100 kHz, 7-bit addresses
 * 
 * @param dev struct to store the I2C base address and device address
 * @param i2c I2C base address
 * @param address device address
 * @param ticks timeout in ticks
 */
void i2c_configure(I2C_Control *dev,uint32_t i2c, uint8_t address);

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, 
 * leave off to use default class value in I2Cdev::readTimeout)
 * @return I2C_Fails
 */
I2C_Fails i2c_read_bit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, 
  uint8_t *data);

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off
 *  to use default class value in I2Cdev::readTimeout)
 * @return I2C_Fails
 */
I2C_Fails i2c_read_byte(I2C_Control *dev, uint8_t regAddr, uint8_t *data);

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off
 *  to use default class value in I2Cdev::readTimeout)
 * @return I2C_Fails
 */
I2C_Fails i2c_read_bytes(I2C_Control *dev, uint8_t regAddr, uint8_t *data, 
  uint8_t length);

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return I2C_Fails
 */
I2C_Fails i2c_write_bit(I2C_Control *dev, uint8_t regAddr, uint8_t bitNum, 
                        uint8_t data);

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return I2C_Fails
 */
I2C_Fails i2c_write_bits(I2C_Control *dev, uint8_t regAddr, uint8_t bitStart, 
                uint8_t length, uint8_t data);

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return I2C_Fails
 */
I2C_Fails i2c_write_byte(I2C_Control *dev, uint8_t regAddr, uint8_t data);

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
void i2c_transfer(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);

/**
 * @brief Reads a given number of bytes from I2C bus
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param res destination buffer to read into
 * @param n number of bytes to read
 */
void i2c_read(uint32_t i2c, int addr, uint8_t *res, size_t n);

/**
 * @brief writes data into I2C bus.
 * 
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param data content to write
 * @param n number of bytes to write 
 */
void i2c_write(uint32_t i2c, int addr, const uint8_t *data, size_t n);

#endif // I2C_H