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

#endif // FAULT_HANDLERS_H
