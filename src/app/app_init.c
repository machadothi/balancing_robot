/**
 * @file app_init.c
 * @brief Application initialization implementation
 *
 * Knows no module by name: app_modules (generated from the CMake
 * configuration) lists the enabled ones.
 *
 * @author Thiago Cunha
 * @date 2024
 */

#include <stddef.h>

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
    for (size_t i = 0; i < app_module_count; i++) {
        if (app_modules[i]->init != NULL) {
            app_modules[i]->init();
        }
    }
}

void app_tasks_init(void) {
    for (size_t i = 0; i < app_module_count; i++) {
        const App_Module_t *module = app_modules[i];
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
