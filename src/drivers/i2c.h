/**
 * @file i2c.h
 * @brief I2C driver for STM32F103
 * 
 * Supports three modes of operation:
 *   - Polling: Blocking transfers (original API)
 *   - Interrupt: Non-blocking with callback notification
 *   - DMA: Hardware-driven transfers with callback
 * 
 * Hardware Configuration:
 *   - I2C1: PB6=SCL, PB7=SDA
 *   - DMA1 Channel 6: I2C1_TX
 *   - DMA1 Channel 7: I2C1_RX
 * 
 * @author Thiago Cunha
 * @date 2023
 */

#ifndef DRIVERS_I2C_H
#define DRIVERS_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Types
 * ========================================================================== */

/**
 * @brief I2C operation result codes
 */
typedef enum {
    I2C_Ok = 0,
    I2C_Addr_Timeout,
    I2C_Write_Timeout,
    I2C_Read_Timeout,
    I2C_Busy_Timeout,
    I2C_Busy,               /**< Transfer in progress */
    I2C_DMA_Error,          /**< DMA transfer error */
    I2C_Bus_Error,          /**< Bus error (misplaced start/stop) */
    I2C_Arbitration_Lost,   /**< Lost arbitration */
    I2C_Nack,               /**< No acknowledge received */
    I2C_Overrun             /**< Overrun/underrun error */
} I2C_Fails_t;

/**
 * @brief I2C transfer mode
 */
typedef enum {
    I2C_MODE_POLLING = 0,   /**< Blocking polling mode */
    I2C_MODE_IT,            /**< Interrupt-driven mode */
    I2C_MODE_DMA            /**< DMA-driven mode */
} I2C_Mode_t;

/**
 * @brief I2C async transfer state
 */
typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_BUSY_TX,
    I2C_STATE_BUSY_RX,
    I2C_STATE_BUSY_TX_RX,   /**< Write then read (repeated start) */
    I2C_STATE_ERROR
} I2C_State_t;

/**
 * @brief I2C transfer direction
 */
typedef enum {
    I2C_DIR_WRITE = 0,
    I2C_DIR_READ
} I2C_Direction_t;

/* Forward declaration */
struct I2C_Control;

/**
 * @brief Callback function type for async transfers
 * @param dev     Pointer to control structure
 * @param result  Transfer result
 */
typedef void (*I2C_Callback_t)(struct I2C_Control *dev, I2C_Fails_t result);

/**
 * @brief I2C control structure (extended for async operations)
 */
typedef struct I2C_Control {
    /* Basic configuration */
    uint32_t device;        /**< I2C peripheral base address */
    uint8_t  addr;          /**< 7-bit device address */
    uint32_t timeout;       /**< Timeout in ticks */
    
    /* Async transfer state */
    volatile I2C_State_t state;     /**< Current transfer state */
    I2C_Mode_t mode;                /**< Current operating mode */
    I2C_Direction_t direction;      /**< Current transfer direction */
    
    /* Transfer buffers */
    const uint8_t *tx_buf;          /**< TX buffer pointer */
    uint8_t *rx_buf;                /**< RX buffer pointer */
    volatile size_t tx_len;         /**< TX remaining bytes */
    volatile size_t rx_len;         /**< RX remaining bytes */
    volatile size_t tx_count;       /**< TX bytes transferred */
    volatile size_t rx_count;       /**< RX bytes transferred */
    
    /* Callback */
    I2C_Callback_t callback;        /**< Completion callback */
    void *user_data;                /**< User context for callback */
    
    /* Error tracking */
    volatile I2C_Fails_t error;     /**< Last error code */
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

/* ==========================================================================
 * Interrupt-Based API
 * ========================================================================== */

/**
 * @brief Initialize I2C interrupts
 * 
 * Enables I2C event and error interrupts in NVIC.
 * Call after i2c_configure().
 * 
 * @param dev       Pointer to control structure
 * @param priority  NVIC priority (0-15, lower = higher priority)
 */
void i2c_init_it(I2C_Control_t *dev, uint8_t priority);

/**
 * @brief Start an interrupt-driven write transfer
 * 
 * @param dev       Pointer to control structure
 * @param data      Data buffer to transmit
 * @param len       Number of bytes to transmit
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_write_it(I2C_Control_t *dev, const uint8_t *data, size_t len,
                         I2C_Callback_t callback);

/**
 * @brief Start an interrupt-driven read transfer
 * 
 * @param dev       Pointer to control structure
 * @param data      Buffer to receive data
 * @param len       Number of bytes to read
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_read_it(I2C_Control_t *dev, uint8_t *data, size_t len,
                        I2C_Callback_t callback);

/**
 * @brief Start an interrupt-driven register read (write reg addr, then read)
 * 
 * Uses repeated start condition between write and read phases.
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Register address to read from
 * @param data      Buffer to receive data
 * @param len       Number of bytes to read
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_read_reg_it(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data,
                            size_t len, I2C_Callback_t callback);

/**
 * @brief Start an interrupt-driven register write
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Register address to write to
 * @param data      Data to write
 * @param len       Number of data bytes
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_write_reg_it(I2C_Control_t *dev, uint8_t regAddr,
                             const uint8_t *data, size_t len,
                             I2C_Callback_t callback);

/* ==========================================================================
 * DMA-Based API
 * ========================================================================== */

/**
 * @brief Initialize I2C with DMA support
 * 
 * Configures DMA channels for I2C transfers:
 *   - I2C1: DMA1 Channel 6 (TX), Channel 7 (RX)
 * 
 * @param dev       Pointer to control structure
 * @param priority  NVIC priority for DMA interrupts
 */
void i2c_init_dma(I2C_Control_t *dev, uint8_t priority);

/**
 * @brief Start a DMA-driven write transfer
 * 
 * @param dev       Pointer to control structure
 * @param data      Data buffer to transmit (must remain valid until complete)
 * @param len       Number of bytes to transmit
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_write_dma(I2C_Control_t *dev, const uint8_t *data, size_t len,
                          I2C_Callback_t callback);

/**
 * @brief Start a DMA-driven read transfer
 * 
 * @param dev       Pointer to control structure
 * @param data      Buffer to receive data (must remain valid until complete)
 * @param len       Number of bytes to read
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_read_dma(I2C_Control_t *dev, uint8_t *data, size_t len,
                         I2C_Callback_t callback);

/**
 * @brief Start a DMA-driven register read
 * 
 * First sends register address via DMA, then reads data.
 * 
 * @param dev       Pointer to control structure
 * @param regAddr   Register address to read from
 * @param data      Buffer to receive data
 * @param len       Number of bytes to read
 * @param callback  Completion callback (can be NULL)
 * @return I2C_Ok if transfer started, I2C_Busy if busy
 */
I2C_Fails_t i2c_read_reg_dma(I2C_Control_t *dev, uint8_t regAddr, uint8_t *data,
                             size_t len, I2C_Callback_t callback);

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

/**
 * @brief Check if a transfer is in progress
 * 
 * @param dev       Pointer to control structure
 * @return true if busy, false if idle
 */
bool i2c_is_busy(I2C_Control_t *dev);

/**
 * @brief Get the current transfer state
 * 
 * @param dev       Pointer to control structure
 * @return Current I2C_State_t
 */
I2C_State_t i2c_get_state(I2C_Control_t *dev);

/**
 * @brief Abort an ongoing async transfer
 * 
 * @param dev       Pointer to control structure
 */
void i2c_abort(I2C_Control_t *dev);

/**
 * @brief Get last error code
 * 
 * @param dev       Pointer to control structure
 * @return Last I2C_Fails_t error
 */
I2C_Fails_t i2c_get_error(I2C_Control_t *dev);

/**
 * @brief Convert error code to string
 * 
 * @param fail      Error code
 * @return String representation
 */
const char* i2c_error_string(I2C_Fails_t fail);

/* ==========================================================================
 * ISR Handlers (called from interrupt vectors)
 * ========================================================================== */

/**
 * @brief I2C Event interrupt handler
 * 
 * Call this from I2C1_EV_IRQHandler or I2C2_EV_IRQHandler.
 * 
 * @param dev       Pointer to control structure
 */
void i2c_ev_isr(I2C_Control_t *dev);

/**
 * @brief I2C Error interrupt handler
 * 
 * Call this from I2C1_ER_IRQHandler or I2C2_ER_IRQHandler.
 * 
 * @param dev       Pointer to control structure
 */
void i2c_er_isr(I2C_Control_t *dev);

/**
 * @brief DMA TX complete handler
 * 
 * Call this from DMA1_Channel6_IRQHandler (for I2C1).
 * 
 * @param dev       Pointer to control structure
 */
void i2c_dma_tx_isr(I2C_Control_t *dev);

/**
 * @brief DMA RX complete handler
 * 
 * Call this from DMA1_Channel7_IRQHandler (for I2C1).
 * 
 * @param dev       Pointer to control structure
 */
void i2c_dma_rx_isr(I2C_Control_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_I2C_H */