#ifndef LOG_H_
#define LOG_H_

typedef enum {
    UART_BUS,
    MPU6050,
    I2C_BUS,
    // Add more modules as needed
} LogModule_t;

typedef enum {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
    OFF
} LogLevel_t;

typedef struct {
    LogLevel_t log_level;
    void (*send)(const char *message);
} LogDriver_t;

/**
 * @brief Configures the log module with the minimum log level and its callback
 * function inside the driver struct.
 * 
 * @param log 
 * @param driver 
 */
void log_init(LogDriver_t *driver);

/**
 * @brief Logs a message with a specific log level and module. 
 * The message is only logged if the log level is greater than or equal 
 * to the configured log level.
 * 
 * @param log log level 
 * @param module module sending the message
 * @param message content
 */
void log_message(LogLevel_t log, LogModule_t module, const char *message);

/**
 * @brief Logs a message with a specific log level, module, and error message. 
 * The message is only logged if the log level is greater than or equal 
 * to the configured log level. The error message is appended to the log message.
 * 
 * @param log log level 
 * @param module module sending the message
 * @param message content
 * @param error error message
 */
void log_message_with_error(LogLevel_t log, LogModule_t module, 
  const char *message, const char *error);

/**
 * @brief Logs a message with a specific log level, module, and integer value. 
 * The message is only logged if the log level is greater than or equal 
 * to the configured log level. The integer value is appended to the log message.
 * 
 * @param log log level 
 * @param module module sending the message
 * @param message content
 * @param value integer value
 */
void log_message_with_int(LogLevel_t log, LogModule_t module, 
  const char *message, int value);


#endif /* LOG_H_ */