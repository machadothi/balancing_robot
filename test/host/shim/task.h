/* Host stand-in for task.h */
#ifndef HOST_TASK_H
#define HOST_TASK_H

#include "FreeRTOS.h"

static inline void vTaskDelay(TickType_t ticks) { (void)ticks; }

#endif // HOST_TASK_H
