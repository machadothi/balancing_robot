#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

// -----------------------------------------------------------------------------
/* ---------------------------- PRIVATE FUNCTIONS ----------------------------*/
// -----------------------------------------------------------------------------

static LogDriver_t *driver_ = NULL;

static const char* get_timestamp(void);
static const char* level_to_string(LogLevel_t level);
static const char* module_to_string(LogModule_t module);

// -----------------------------------------------------------------------------
/* ---------------------------- PUBLIC FUNCTIONS -----------------------------*/
// -----------------------------------------------------------------------------

void log_init(LogDriver_t *driver) {
    driver_ = driver;
}

// -----------------------------------------------------------------------------

void log_message(LogLevel_t log, LogModule_t module, const char *message) {
    if (driver_ == NULL || message == NULL) {
        return;
    }

    if (log >= driver_->log_level) {
        char log_message[256];
        sprintf(log_message, "[T: %s|%s][%s] %s\n\r", get_timestamp(), 
          module_to_string(module), level_to_string(log), message);
        
        driver_->send(log_message);
    }
}

// -----------------------------------------------------------------------------

void log_message_with_error(LogLevel_t log, LogModule_t module, 
  const char *message, const char *error) {
    if (driver_ == NULL || message == NULL || error == NULL) {
        return;
    }

    if (log >= driver_->log_level) {
        char log_message[256];
        sprintf(log_message, "[T: %s|%s][%s] %s. Error: %s\n\r", get_timestamp(), 
          module_to_string(module), level_to_string(log), message, error);
        
        driver_->send(log_message);
    }
}

// -----------------------------------------------------------------------------

void log_message_with_int(LogLevel_t log, LogModule_t module, 
  const char *message, int value) {
    if (driver_ == NULL || message == NULL) {
        return;
    }

    if (log >= driver_->log_level) {
        char log_message[256];
        char value_as_hex[10];
        sprintf(value_as_hex, "0x%X", value);
        sprintf(log_message, "[T: %s|%s][%s] %s. Value: %s\n\r", get_timestamp(), 
          module_to_string(module), level_to_string(log), message, value_as_hex);
        
        driver_->send(log_message);
    }
}


// -----------------------------------------------------------------------------
/* ---------------------------- PRIVATE FUNCTIONS ----------------------------*/
// -----------------------------------------------------------------------------

static const char* module_to_string(LogModule_t module) {
    switch (module) {
        case UART_BUS:
            return "UART MODULE";
        case MPU6050:
            return "IMU MODULE";
        case I2C_BUS:
            return "I2C MODULE";
        // Add more cases as needed for additional interfaces
        default:
            return "Unknown";
    }
}

// -----------------------------------------------------------------------------

static const char* level_to_string(LogLevel_t level) {
    switch (level) {
        case DEBUG:
            return "DEBUG";
        case INFO:
            return "INFO";
        case WARN:
            return "WARN";
        case ERROR:
            return "ERROR";
        case FATAL:
            return "FATAL";
        default:
            return "Unknown";
    }
}

// -----------------------------------------------------------------------------

static const char* get_timestamp() {
    TickType_t ticks = xTaskGetTickCount();
    static char timestamp[20];
    sprintf(timestamp, "0x%lX", ticks);
    return timestamp;
}
