/**
 * @file fault_handlers.h
 * @brief Fault and error handlers for FreeRTOS and Cortex-M3
 * 
 * @author Thiago Cunha
 * @date 2024
 */

#ifndef FAULT_HANDLERS_H
#define FAULT_HANDLERS_H

#include <FreeRTOS.h>
#include <task.h>

/**
 * @brief Hard fault handler - called on CPU exceptions
 */
void hard_fault_handler(void);

/**
 * @brief FreeRTOS malloc failed hook
 */
void vApplicationMallocFailedHook(void);

/* FreeRTOS V10+ already declares the stack overflow hook in task.h */
#if tskKERNEL_VERSION_MAJOR < 10
/**
 * @brief FreeRTOS stack overflow hook
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
#endif

#endif /* FAULT_HANDLERS_H */
