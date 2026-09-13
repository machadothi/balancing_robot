/**
 * @file log.h
 * @brief Logging system for embedded applications
 * 
 * Provides structured logging with multiple severity levels and
 * module tagging. Output is configurable via driver abstraction.
 * 
 * Usage:
 * @code
 * LogDriver_t driver = {
 *     .log_level = INFO,
 *     .send = uart_puts
 * };
 * log_init(&driver);
 * log_message(LOG_INFO, IMU_TASK, "Sensor initialized");
 * @endcode
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef LOG_H
#define LOG_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* ==========================================================================
 * Type Definitions
 * ========================================================================== */

/**
 * @brief Log module identifiers
 * 
 * Add new modules here as needed for your application.
 */
typedef enum {
    UART_BUS,       /**< UART communication module */
    I2C_BUS,        /**< I2C communication module */
    IMU_TASK,       /**< IMU data acquisition task */
    MPU6050,        /**< MPU6050 sensor driver */
    ROBOT_TASK,     /**< Robot control task */
    MOTOR_TASK,     /**< Motor control module */
    /* Add more modules as needed */
} LogModule_t;

/**
 * @brief Log severity levels
 * 
 * Messages are only output if their level >= configured minimum level.
 */
typedef enum {
    LOG_DEBUG,      /**< Detailed debug information */
    LOG_INFO,       /**< General information */
    LOG_WARN,       /**< Warning conditions */
    LOG_ERROR,      /**< Error conditions */
    LOG_FATAL,      /**< Fatal errors */
    LOG_OFF         /**< Disable all logging */
} LogLevel_t;

/**
 * @brief Log driver configuration
 */
typedef struct {
    LogLevel_t log_level;               /**< Minimum level to output */
    void (*send)(const char *message);  /**< Output function */
} LogDriver_t;

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

/**
 * @brief Initialize the logging system
 * @param driver Pointer to configured log driver
 */
void log_init(LogDriver_t *driver);

/**
 * @brief Log a message
 * @param level Severity level
 * @param module Source module
 * @param message Message text
 */
void log_message(LogLevel_t level, LogModule_t module, const char *message);

/**
 * @brief Log a message with error detail
 * @param level Severity level
 * @param module Source module
 * @param message Message text
 * @param error Error description
 */
void log_message_with_error(LogLevel_t level, LogModule_t module, 
    const char *message, const char *error);

/**
 * @brief Log a message with integer value
 * @param level Severity level
 * @param module Source module
 * @param message Message text
 * @param value Integer value to append
 */
void log_message_with_int(LogLevel_t level, LogModule_t module, 
    const char *message, int value);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // LOG_H
