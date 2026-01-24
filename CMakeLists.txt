# =============================================================================
# Balancing Robot - Main CMake Configuration
# =============================================================================
# Self-contained build for STM32F103C8T6 self-balancing robot
# 
# Dependencies (as git submodules):
#   - libopencm3: Open-source ARM Cortex-M firmware library
#   - FreeRTOS-Kernel: Real-time operating system
#
# Build instructions:
#   mkdir build && cd build
#   cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
#   make
#
# Author: Thiago Cunha
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# Project definition
project(balancing-robot
    VERSION 1.0.0
    DESCRIPTION "Self-balancing robot with STM32F103C8T6 and FreeRTOS"
    LANGUAGES C ASM
)

# =============================================================================
# Build Configuration
# =============================================================================

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Target MCU configuration
set(MCU_FAMILY STM32F1)
set(MCU_MODEL STM32F103C8T6)
set(MCU_FLASH_SIZE 64K)
set(MCU_RAM_SIZE 20K)

# Paths
set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/lib/libopencm3)
set(FREERTOS_DIR ${CMAKE_SOURCE_DIR}/src/rtos)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Linker script
set(LINKER_SCRIPT ${SRC_DIR}/stm32f103c8t6.ld)

# =============================================================================
# Check Dependencies
# =============================================================================

if(NOT EXISTS ${LIBOPENCM3_DIR}/include)
    message(FATAL_ERROR 
        "libopencm3 not found!\n"
        "Run './scripts/setup.sh' to initialize the project, or:\n"
        "  git submodule update --init --recursive\n"
        "  cd lib/libopencm3 && make TARGETS=stm32/f1"
    )
endif()

if(NOT EXISTS ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f1.a)
    message(FATAL_ERROR 
        "libopencm3 not built!\n"
        "Run: cd lib/libopencm3 && make TARGETS=stm32/f1"
    )
endif()

# =============================================================================
# Compiler Flags
# =============================================================================

# Warning flags
add_compile_options(
    -Wall
    -Wextra
    -Wshadow
    -Wundef
    -Wimplicit-function-declaration
    -Wredundant-decls
    -Wmissing-prototypes
    -Wstrict-prototypes
)

# Optimization and debug flags
add_compile_options(
    -Os
    -g
    -fno-common
    -ffunction-sections
    -fdata-sections
)

# MCU definition
add_compile_definitions(${MCU_FAMILY})

# =============================================================================
# FreeRTOS Library (bundled with project, pre-configured for libopencm3)
# =============================================================================

add_library(freertos STATIC
    # Core FreeRTOS files
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    
    # Memory management (heap_4 for fragmentation-resistant allocation)
    ${FREERTOS_DIR}/heap_4.c
    
    # ARM Cortex-M3 port
    ${FREERTOS_DIR}/port.c
)
# NOTE: opencm3.c is linked with app (not library) to ensure interrupt handlers
# are included - static library would drop unreferenced symbols

target_include_directories(freertos PUBLIC
    ${FREERTOS_DIR}
    ${SRC_DIR}  # For FreeRTOSConfig.h
    ${LIBOPENCM3_DIR}/include
)

# =============================================================================
# Application Sources
# =============================================================================

set(APP_SOURCES
    ${SRC_DIR}/main.c
    ${SRC_DIR}/interrupts.c
    ${SRC_DIR}/drivers/i2c.c
    ${SRC_DIR}/drivers/uart.c
    ${SRC_DIR}/filter/kalman.c
    ${SRC_DIR}/filter/complementary.c
    ${SRC_DIR}/imu/imu.c
    ${SRC_DIR}/imu/mpu6050.c
    ${SRC_DIR}/led/led.c
    ${SRC_DIR}/log/log.c
    ${SRC_DIR}/motor/motor.c
    ${SRC_DIR}/robot/robot.c
    # FreeRTOS interrupt handlers - must be in app to ensure linkage
    ${FREERTOS_DIR}/opencm3.c
)

# =============================================================================
# Main Executable
# =============================================================================

add_executable(${PROJECT_NAME}.elf ${APP_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${SRC_DIR}
    ${FREERTOS_DIR}
    ${LIBOPENCM3_DIR}/include
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    freertos
    ${LIBOPENCM3_DIR}/lib/libopencm3_stm32f1.a
    m  # math library
)

# Linker flags
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    --static
    -nostartfiles
    -specs=nano.specs
    -specs=nosys.specs
    -u _printf_float  # Enable float printf
)

# =============================================================================
# Post-build: Generate .bin and .hex files
# =============================================================================

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Generating ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
)

# =============================================================================
# Flash Target
# =============================================================================

find_program(STFLASH st-flash)

if(STFLASH)
    add_custom_target(flash
        COMMAND ${STFLASH} write ${PROJECT_NAME}.bin 0x8000000
        DEPENDS ${PROJECT_NAME}.elf
        COMMENT "Flashing ${PROJECT_NAME}.bin to STM32"
    )
else()
    message(STATUS "st-flash not found - 'flash' target will not be available")
endif()

# =============================================================================
# Clean Target (removes generated files)
# =============================================================================

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES
    ${PROJECT_NAME}.bin
    ${PROJECT_NAME}.hex
    ${PROJECT_NAME}.map
)

# =============================================================================
# Information
# =============================================================================

message(STATUS "")
message(STATUS "=== Balancing Robot Build Configuration ===")
message(STATUS "MCU:            ${MCU_MODEL}")
message(STATUS "Flash:          ${MCU_FLASH_SIZE}")
message(STATUS "RAM:            ${MCU_RAM_SIZE}")
message(STATUS "Toolchain:      ${CMAKE_C_COMPILER}")
message(STATUS "libopencm3:     ${LIBOPENCM3_DIR}")
message(STATUS "FreeRTOS:       ${FREERTOS_DIR}")
message(STATUS "Linker script:  ${LINKER_SCRIPT}")
message(STATUS "")
