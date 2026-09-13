/**
 * @file app_init.c
 * @brief Application initialization implementation
 *
 * Knows no module by name: the linker collects every APP_MODULE() descriptor
 * that was built into one table (see app/module.h).
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <FreeRTOS.h>
#include <task.h>

#include "config.h"
#include "app/app_init.h"
#include "app/module.h"
#include "board/board.h"

void app_hardware_init(void) {
    board_clock_init();

    /* Every init runs before any task exists: queues and locks are ready
     * whichever task starts first */
    for (const App_Module_t *module = __app_modules_start; module < __app_modules_end; module++) {
        if (module->init != NULL) {
            module->init();
        }
    }
}

void app_tasks_init(void) {
    for (const App_Module_t *module = __app_modules_start; module < __app_modules_end; module++) {
        if (module->task == NULL) {
            continue;
        }

        configASSERT(module->priority < configMAX_PRIORITIES);
        BaseType_t created = xTaskCreate(module->task, module->name, module->stack,
                                         NULL, module->priority, NULL);
        configASSERT(created == pdPASS);
        (void)created;
    }
}
