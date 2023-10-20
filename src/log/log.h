#ifndef LOG_H_
#define LOG_H_

typedef enum {
    UART_BUS,
    MPU6050,
    I2C_BUS,
    // Add more modules as needed
} LogModule_t;

typedef struct {
    void (*send)(const char *message);
} LogDriver_t;

void log_message(LogModule_t module, const char *message, LogDriver_t *driver);
#endif /* LOG_H_ */