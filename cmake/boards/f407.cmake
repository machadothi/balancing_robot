# Hiwonder ROS Robot Control Board: STM32F407VET6, Cortex-M4F,
# 512K flash / 128K SRAM (+64K CCM, unused), 8 MHz HSE

set(MCU_FAMILY STM32F4)
set(MCU_MODEL STM32F407VET6)
set(MCU_FLASH_SIZE 512K)
set(MCU_RAM_SIZE 128K)

set(LIBOPENCM3_TARGET stm32/f4)
set(LIBOPENCM3_LIB ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f4.a)
set(LINKER_SCRIPT ${SRC_DIR}/stm32f407vet6.ld)
# Must match libopencm3's F4 build (hard-float ABI)
set(CPU_FLAGS -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16)
set(BOARD_SOURCE ${SRC_DIR}/board/board_f407.c)
set(BOARD_INCLUDE_DIR ${SRC_DIR}/board/f407)
set(MOTOR_SOURCE ${SRC_DIR}/motor/motor_hiwonder.c)

set(SYS_CLOCK_HZ 168000000)

# FreeRTOS-Kernel submodule (V10.4.3 LTS), Cortex-M4F port
set(FREERTOS_KERNEL_DIR ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel)
set(FREERTOS_SOURCES
    ${FREERTOS_KERNEL_DIR}/tasks.c
    ${FREERTOS_KERNEL_DIR}/queue.c
    ${FREERTOS_KERNEL_DIR}/list.c
    ${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F/port.c
)
set(FREERTOS_INCLUDE_DIRS
    ${FREERTOS_KERNEL_DIR}/include
    ${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F
)

set(BOARD_DEFAULT_HEAP_SIZE 32768)

# Optional hardware this board provides (see cmake/features.cmake)
set(BOARD_HAS_BT_UART ON)
