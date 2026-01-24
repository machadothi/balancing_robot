/**
 * @file i2c.h
 * @brief I2C driver for STM32F103
 * 
 * @author Thiago Cunha
 * @date 2023
 */

#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <stdbool.h>
#include <stdint.h>

#include <libopencm3/stm32/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Types
 * ========================================================================== */

typedef enum {
    I2C_Ok = 0,
    I2C_Addr_Timeout,
    I2C_Write_Timeout,
    I2C_Read_Timeout,
    I2C_Busy_Timeout
} I2C_Fails_t;

typedef struct {
    uint32_t device;        /**< I2C peripheral base address */
    uint8_t  addr;          /**< 7-bit device address */
    uint32_t timeout;       /**< Timeout in ticks */
} I2C_Control_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize I2C peripheral GPIO and clocks
 */
void i2c_setup_peripheral(void);

/**
 * @brief Recover I2C bus from stuck state
 * 
 * Toggles SCL to release a slave holding SDA low.
 * Called automatically by i2c_setup_peripheral().
 */
void i2c_bus_recovery(void);

/**
 * @brief Configure I2C device for 100 kHz, 7-bit addresses
 * 
 * @param dev       Pointer to control structure
 * @param i2c       I2C peripheral base address (e.g., I2C1)
 * @param address   7-bit device address
 * @param timeout   Timeout in ticks
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_configure(I2C_Control_t *dev, uint32_t i2c, uint8_t address,
                          uint32_t timeout);

/**
 * @brief Read a single bit from a register
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Register address
 * @param bitNum    Bit position (0-7)
 * @param data      Output: bit value (0 or 1)
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_read_bit(I2C_Control_t *dev, uint8_t regAddr, uint8_t bitNum, 
                         uint8_t *data);

/**
 * @brief Read a single byte from a register
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Register address
 * @param data      Output: byte value
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_read_byte(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data);

/**
 * @brief Read multiple bytes from a register
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Starting register address
 * @param data      Output buffer
 * @param length    Number of bytes to read
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_read_bytes(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data, 
                           uint8_t length);

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_write_bit(I2C_Control_t *dev, uint8_t regAddr, uint8_t bitNum, 
                        uint8_t data);

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_write_bits(I2C_Control_t *dev, uint8_t regAddr, uint8_t bitStart, 
                uint8_t length, uint8_t data);

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return I2C_Fails_t
 */
I2C_Fails_t i2c_write_byte(I2C_Control_t *dev, uint8_t regAddr, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_I2C_H */