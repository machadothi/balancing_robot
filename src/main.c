/* Simple LED task demo:
 *
 * The LED on PC13 is toggled in task1.
 */
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

extern void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName );

void
vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName ) {
	(void)xTask;
	(void)pcTaskName;
	for(;;);
}

static void
task1(void *args) {
	(void)args;

	for (;;) {
		TickType_t LastWakeTime = xTaskGetTickCount();

		gpio_toggle(GPIOC,GPIO13);
		vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(250));
	}
}

int
main(void) {

	rcc_clock_setup_in_hse_8mhz_out_72mhz();	// Use this for "blue pill"
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

	xTaskCreate(task1,"LED",100,NULL,configMAX_PRIORITIES-1,NULL);
	vTaskStartScheduler();
	for (;;);

	return 0;
}

// End
