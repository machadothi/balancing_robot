/**
 * @file module.h
 * @brief Application module descriptor, collected by the linker
 *
 * A module defines one descriptor in its own .c file:
 *
 * @code
 * APP_MODULE(imu_module) = {
 *     .name = "IMU", .init = imu_queue_init, .task = imu_task,
 *     .stack = 192, .priority = APP_PRIORITY_CONTROL,
 * };
 * @endcode
 *
 * APP_MODULE() puts it in the section ".app_modules.imu_module". Both linker
 * scripts gather every such section, sorted by name, into one array between
 * __app_modules_start and __app_modules_end. A module is therefore in the
 * table exactly when its .c file is built: there is no list to maintain.
 * app_init.c runs all inits, then creates all tasks.
 */

#ifndef APP_MODULE_H
#define APP_MODULE_H

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

/**
 * @brief Define a module descriptor in the module table section
 *
 * `used` keeps the compiler from dropping the unreferenced object; KEEP in
 * the linker script does the same for --gc-sections.
 */
#define APP_MODULE(symbol)                                                      \
    const App_Module_t symbol                                                   \
        __attribute__((used, section(".app_modules." #symbol)))

/** Bounds of the module table, defined by the linker script */
extern const App_Module_t __app_modules_start[];
extern const App_Module_t __app_modules_end[];

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // APP_MODULE_H
