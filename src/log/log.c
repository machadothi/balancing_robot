/**
 * @file log.c
 * @brief Logging system implementation
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#include <stdlib.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "log/log.h"

#if LOGGING

/* ==========================================================================
 * Private Variables
 * ========================================================================== */

static LogDriver_t *driver_ = NULL;

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

static const char *get_timestamp(void);
static const char *level_to_string(LogLevel_t level);
static const char *module_to_string(LogModule_t module);

/* ==========================================================================
 * Public Functions
 * ========================================================================== */

void log_init(LogDriver_t *driver) {
    driver_ = driver;
}

void log_message(LogLevel_t level, LogModule_t module, const char *message) {
    if (driver_ == NULL || message == NULL) {
        return;
    }

    if (level >= driver_->log_level) {
        char buffer[LOG_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "[T:%s|%s][%s] %s\r\n", 
            get_timestamp(), module_to_string(module), 
            level_to_string(level), message);
        driver_->send(buffer);
    }
}

void log_message_with_error(LogLevel_t level, LogModule_t module, 
    const char *message, const char *error) {
    if (driver_ == NULL || message == NULL || error == NULL) {
        return;
    }

    if (level >= driver_->log_level) {
        char buffer[LOG_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "[T:%s|%s][%s] %s. Error: %s\r\n", 
            get_timestamp(), module_to_string(module), 
            level_to_string(level), message, error);
        driver_->send(buffer);
    }
}

void log_message_with_int(LogLevel_t level, LogModule_t module, 
    const char *message, int value) {
    if (driver_ == NULL || message == NULL) {
        return;
    }

    if (level >= driver_->log_level) {
        char buffer[LOG_BUFFER_SIZE];
        snprintf(buffer, sizeof(buffer), "[T:%s|%s][%s] %s: 0x%X\r\n", 
            get_timestamp(), module_to_string(module), 
            level_to_string(level), message, value);
        driver_->send(buffer);
    }
}

/* ==========================================================================
 * Private Functions
 * ========================================================================== */

static const char *module_to_string(LogModule_t module) {
    switch (module) {
        case UART_BUS:    return "UART";
        case I2C_BUS:     return "I2C";
        case IMU_TASK:    return "IMU";
        case MPU6050:     return "MPU";
        case ROBOT_TASK:  return "ROBOT";
        case MOTOR_TASK:  return "MOTOR";
        default:          return "???";
    }
}

static const char *level_to_string(LogLevel_t level) {
    switch (level) {
        case LOG_DEBUG:   return "DBG";
        case LOG_INFO:    return "INF";
        case LOG_WARN:    return "WRN";
        case LOG_ERROR:   return "ERR";
        case LOG_FATAL:   return "FTL";
        default:          return "???";
    }
}

static const char *get_timestamp(void) {
    static char timestamp[16];
    TickType_t ticks = xTaskGetTickCount();
    uint32_t ms = (uint32_t)(ticks * portTICK_PERIOD_MS);
    snprintf(timestamp, sizeof(timestamp), "%lu", (unsigned long)ms);
    return timestamp;
}

#else /* LOGGING == 0 */

/* Stub implementations when logging is disabled */
void log_init(LogDriver_t *driver) { (void)driver; }
void log_message(LogLevel_t level, LogModule_t module, const char *message) {
    (void)level; (void)module; (void)message;
}
void log_message_with_error(LogLevel_t level, LogModule_t module,
    const char *message, const char *error) {
    (void)level; (void)module; (void)message; (void)error;
}
void log_message_with_int(LogLevel_t level, LogModule_t module,
    const char *message, int value) {
    (void)level; (void)module; (void)message; (void)value;
}

#endif // LOGGING
