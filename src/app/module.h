/**
 * @file module.h
 * @brief Application module descriptor
 *
 * A module exports one descriptor from its own .c file:
 *
 * @code
 * const App_Module_t imu_module = {
 *     .name = "IMU", .init = imu_queue_init, .task = imu_task,
 *     .stack = 192, .priority = APP_PRIORITY_CONTROL,
 * };
 * @endcode
 *
 * and CMake lists it (robot_add_module(imu_module), or MODULES in
 * robot_feature). The generated app_modules.c holds every enabled module;
 * app_init.c runs all inits, then creates all tasks.
 */

#ifndef APP_MODULE_H
#define APP_MODULE_H

#include <stddef.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/* Priorities (configMAX_PRIORITIES = 5): the sensing/control chain must never
 * wait behind console I/O, and background work runs only when nothing else does */
#define APP_PRIORITY_CONTROL        4   /**< IMU and balance control */
#define APP_PRIORITY_IO             2   /**< AT commands and telemetry */
#define APP_PRIORITY_BACKGROUND     1   /**< Heartbeat LED */

typedef struct {
    const char *name;           /**< Task name (configMAX_TASK_NAME_LEN) */
    void (*init)(void);         /**< Before the scheduler starts; NULL if none */
    TaskFunction_t task;        /**< NULL for a module without a task */
    uint16_t stack;             /**< Task stack, in words */
    UBaseType_t priority;       /**< APP_PRIORITY_* */
} App_Module_t;

/** Enabled modules, generated from the CMake configuration */
extern const App_Module_t *const app_modules[];
extern const size_t app_module_count;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // APP_MODULE_H
