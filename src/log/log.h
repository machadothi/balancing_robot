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
    FATAL
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

void log_message(LogLevel_t log, LogModule_t module, const char *message);

#endif /* LOG_H_ */