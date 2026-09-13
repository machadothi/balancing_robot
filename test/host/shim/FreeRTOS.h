/* Host stand-in for FreeRTOS.h: only what the tested sources use */
#ifndef HOST_FREERTOS_H
#define HOST_FREERTOS_H

#include <assert.h>
#include <stdint.h>

typedef uint32_t TickType_t;
typedef long BaseType_t;
typedef unsigned long UBaseType_t;

#define pdMS_TO_TICKS(ms)   ((TickType_t)(ms))
#define configASSERT(x)     assert(x)

#endif // HOST_FREERTOS_H
