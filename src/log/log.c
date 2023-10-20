#include <stdlib.h>

#include "log.h"

static const char* module_to_string(LogModule_t module);

void log_message(LogModule_t module, const char *message, LogDriver_t *driver) {
    if (driver == NULL || message == NULL) {
        return;
    }

    switch (module) {
        case UART_BUS:
            // Add specific logging behavior for MODULE_A if needed
            break;
        case MPU6050:
            // Add specific logging behavior for MODULE_B if needed
            break;
        case I2C_BUS:
            // Add specific logging behavior for MODULE_C if needed
            break;
        // Add more cases as needed for additional modules
        default:
            break;
    }
    driver->send(module_to_string(module));
    driver->send(message);
    driver->send("\n\r");
}

static const char* module_to_string(LogModule_t module) {
    switch (module) {
        case UART_BUS:
            return "[UART]: ";
        case MPU6050:
            return "[IMU]: ";
        case I2C_BUS:
            return "[I2C]: ";
        // Add more cases as needed for additional interfaces
        default:
            return "Unknown";
    }
}

