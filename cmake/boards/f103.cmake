# Blue Pill: STM32F103C8T6, Cortex-M3, 64K flash / 20K RAM, 8 MHz HSE

set(MCU_FAMILY STM32F1)
set(MCU_MODEL STM32F103C8T6)
set(MCU_FLASH_SIZE 64K)
set(MCU_RAM_SIZE 20K)

set(LIBOPENCM3_TARGET stm32/f1)
set(LIBOPENCM3_LIB ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f1.a)
set(LINKER_SCRIPT ${SRC_DIR}/stm32f103c8t6.ld)
set(CPU_FLAGS -mthumb -mcpu=cortex-m3 -msoft-float -mfix-cortex-m3-ldrd)
set(BOARD_SOURCE ${SRC_DIR}/board/board_f103.c)
set(BOARD_INCLUDE_DIR ${SRC_DIR}/board/f103)
set(MOTOR_SOURCE ${SRC_DIR}/motor/motor.c)

set(SYS_CLOCK_HZ 72000000)

# Vendored FreeRTOS V9, Cortex-M3 port
set(FREERTOS_SOURCES
    ${SRC_DIR}/rtos/tasks.c
    ${SRC_DIR}/rtos/queue.c
    ${SRC_DIR}/rtos/list.c
    ${SRC_DIR}/rtos/heap_4.c
    ${SRC_DIR}/rtos/port.c
)
set(FREERTOS_INCLUDE_DIRS ${SRC_DIR}/rtos)

set(BOARD_DEFAULT_HEAP_SIZE 12288)
set(BOARD_DEFAULT_APP_BLINK_ONLY OFF)
