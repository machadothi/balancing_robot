# =============================================================================
# Balancing Robot - Main CMake Configuration
# =============================================================================
# Self-contained build for the STM32 self-balancing robot.
#
# Supported boards (BOARD cache variable, see cmake/boards/):
#   - f103: Blue Pill (STM32F103C8T6)
#   - f407: Hiwonder ROS Robot Control Board (STM32F407VET6)
#
# Dependencies (as git submodules):
#   - libopencm3: Open-source ARM Cortex-M firmware library
#   - FreeRTOS-Kernel: Real-time operating system (F407 build)
#
# Build instructions:
#   cmake --preset f103        (or f407)
#   cmake --build --preset f103
#
# Author: Thiago Cunha
# =============================================================================

cmake_minimum_required(VERSION 3.21)

# Project definition
project(balancing-robot
    VERSION 1.0.0
    DESCRIPTION "Self-balancing robot with STM32 and FreeRTOS"
    LANGUAGES C ASM
)

# =============================================================================
# Build Configuration
# =============================================================================

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Paths
set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/lib/libopencm3)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)

# =============================================================================
# Board Selection
# =============================================================================

set(BOARDS f103 f407)
set(BOARD f103 CACHE STRING "Target board (${BOARDS})")
set_property(CACHE BOARD PROPERTY STRINGS ${BOARDS})

if(NOT BOARD IN_LIST BOARDS)
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Valid boards: ${BOARDS}")
endif()

# Sets MCU_*, LIBOPENCM3_*, LINKER_SCRIPT, CPU_FLAGS, FREERTOS_*,
# SYS_CLOCK_HZ, BOARD_SOURCE and BOARD_DEFAULT_* for the options below
include(${CMAKE_SOURCE_DIR}/cmake/boards/${BOARD}.cmake)

# =============================================================================
# Application Options (written to generated/app_config.h)
# =============================================================================

include(CMakeDependentOption)

option(APP_BLINK_ONLY "Only run the LED heartbeat task (no drivers)"
    ${BOARD_DEFAULT_APP_BLINK_ONLY})

set(UART_BAUDRATE 921600 CACHE STRING "UART baud rate")
set(IMU_SAMPLE_RATE_MS 10 CACHE STRING "IMU sample period in ms")
set(FREERTOS_TICK_RATE_HZ 250 CACHE STRING "FreeRTOS tick rate in Hz")
set(FREERTOS_TOTAL_HEAP_SIZE ${BOARD_DEFAULT_HEAP_SIZE} CACHE STRING
    "FreeRTOS heap size in bytes")

foreach(value UART_BAUDRATE IMU_SAMPLE_RATE_MS FREERTOS_TICK_RATE_HZ FREERTOS_TOTAL_HEAP_SIZE)
    if(NOT ${value} MATCHES "^[0-9]+$")
        message(FATAL_ERROR "${value} must be a positive integer, got '${${value}}'")
    endif()
endforeach()

# Driver features: forced OFF when the drivers are not built
cmake_dependent_option(UART_PRINTF_ENABLED "Enable uart_printf" ON "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(UART_ECHO_ENABLED "Echo received UART characters" ON "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(I2C_DMA_ENABLED "I2C DMA transfers (required for IMU)" ON "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(I2C_BUS_RECOVERY "I2C bus recovery" ON "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(AT_CMD_HELP_ENABLED "AT+HELP command" OFF "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(AT_CMD_ALL_QUERY "AT+ALL? bulk query" OFF "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(AT_CMD_PID_TOGGLE "AT+PIDON/PIDOFF/PID commands" ON "NOT APP_BLINK_ONLY" OFF)
cmake_dependent_option(LOG_ENABLED "Logging over UART (~300 bytes stack)" OFF "NOT APP_BLINK_ONLY" OFF)
# Verbose fault output needs an initialized UART
cmake_dependent_option(FAULT_HANDLERS_VERBOSE "Print fault messages over UART" ON "NOT APP_BLINK_ONLY" OFF)

if(BOARD STREQUAL "f407" AND NOT APP_BLINK_ONLY)
    message(FATAL_ERROR
        "The F407 drivers (UART, I2C, PWM) are not ported yet.\n"
        "Configure with -DAPP_BLINK_ONLY=ON."
    )
endif()

configure_file(${SRC_DIR}/app_config.h.in ${GENERATED_DIR}/app_config.h)

# =============================================================================
# Check Dependencies
# =============================================================================

if(NOT EXISTS ${LIBOPENCM3_DIR}/include)
    message(FATAL_ERROR
        "libopencm3 not found!\n"
        "Run './scripts/setup.sh' to initialize the project, or:\n"
        "  git submodule update --init --recursive\n"
        "  cd lib/libopencm3 && make TARGETS=${LIBOPENCM3_TARGET}"
    )
endif()

if(NOT EXISTS ${LIBOPENCM3_LIB})
    message(FATAL_ERROR
        "libopencm3 not built for ${MCU_FAMILY}!\n"
        "Run: cd lib/libopencm3 && make TARGETS=${LIBOPENCM3_TARGET}"
    )
endif()

list(GET FREERTOS_SOURCES 0 FREERTOS_FIRST_SOURCE)
if(NOT EXISTS ${FREERTOS_FIRST_SOURCE})
    message(FATAL_ERROR
        "FreeRTOS sources not found!\n"
        "Run: git submodule update --init lib/FreeRTOS-Kernel"
    )
endif()

# =============================================================================
# Compiler Flags
# =============================================================================

add_compile_options(${CPU_FLAGS})
add_link_options(${CPU_FLAGS})

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
# FreeRTOS Library
# =============================================================================

add_library(freertos STATIC ${FREERTOS_SOURCES})
# NOTE: src/rtos/opencm3.c is linked with app (not library) to ensure interrupt
# handlers are included - static library would drop unreferenced symbols

target_include_directories(freertos PUBLIC
    ${FREERTOS_INCLUDE_DIRS}
    ${SRC_DIR}  # For FreeRTOSConfig.h
    ${GENERATED_DIR}
    ${LIBOPENCM3_DIR}/include
)

# =============================================================================
# Application Sources
# =============================================================================

set(APP_SOURCES
    ${SRC_DIR}/main.c
    ${SRC_DIR}/app/app_init.c
    ${SRC_DIR}/fault/fault_handlers.c
    ${SRC_DIR}/led/led.c
    ${SRC_DIR}/log/log.c
    ${BOARD_SOURCE}
    # FreeRTOS interrupt handlers - must be in app to ensure linkage
    ${SRC_DIR}/rtos/opencm3.c
)

if(NOT APP_BLINK_ONLY)
    list(APPEND APP_SOURCES
        ${SRC_DIR}/interrupts.c
        ${SRC_DIR}/cmd/at_cmd.c
        ${SRC_DIR}/drivers/i2c.c
        ${SRC_DIR}/drivers/pwm.c
        ${SRC_DIR}/drivers/uart.c
        ${SRC_DIR}/filter/kalman.c
        ${SRC_DIR}/filter/complementary.c
        ${SRC_DIR}/imu/imu.c
        ${SRC_DIR}/imu/mpu6050.c
        ${SRC_DIR}/motor/motor.c
        ${SRC_DIR}/robot/robot.c
    )
endif()

# =============================================================================
# Main Executable
# =============================================================================

add_executable(${PROJECT_NAME}.elf ${APP_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${SRC_DIR}
    ${GENERATED_DIR}
    ${LIBOPENCM3_DIR}/include
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    freertos
    ${LIBOPENCM3_LIB}
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

set_property(TARGET ${PROJECT_NAME}.elf APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})

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
# Flash Targets
# =============================================================================

find_program(STFLASH st-flash)

if(STFLASH)
    add_custom_target(flash
        COMMAND ${STFLASH} write ${PROJECT_NAME}.bin 0x8000000
        DEPENDS ${PROJECT_NAME}.elf
        COMMENT "Flashing ${PROJECT_NAME}.bin to ${MCU_MODEL} over ST-Link"
    )
else()
    message(STATUS "st-flash not found - 'flash' target will not be available")
endif()

set(SERIAL_PORT /dev/ttyUSB0 CACHE STRING "Serial port for flash-serial")
# stm32flash -i syntax: <enter sequence>:<exit sequence>
set(SERIAL_BOOT_SEQUENCE "-dtr&rts,dtr:-rts" CACHE STRING
    "DTR/RTS sequence stm32flash uses to enter the ROM bootloader")

find_program(STM32FLASH stm32flash)

if(STM32FLASH)
    add_custom_target(flash-serial
        COMMAND ${STM32FLASH} -w ${PROJECT_NAME}.bin -v -g 0x0
            -i ${SERIAL_BOOT_SEQUENCE} ${SERIAL_PORT}
        DEPENDS ${PROJECT_NAME}.elf
        COMMENT "Flashing ${PROJECT_NAME}.bin to ${MCU_MODEL} over ${SERIAL_PORT}"
        VERBATIM
    )
else()
    message(STATUS "stm32flash not found - 'flash-serial' target will not be available")
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
message(STATUS "Board:          ${BOARD}")
message(STATUS "MCU:            ${MCU_MODEL}")
message(STATUS "Flash:          ${MCU_FLASH_SIZE}")
message(STATUS "RAM:            ${MCU_RAM_SIZE}")
message(STATUS "Blink only:     ${APP_BLINK_ONLY}")
message(STATUS "Toolchain:      ${CMAKE_C_COMPILER}")
message(STATUS "libopencm3:     ${LIBOPENCM3_LIB}")
message(STATUS "FreeRTOS:       ${FREERTOS_INCLUDE_DIRS}")
message(STATUS "Linker script:  ${LINKER_SCRIPT}")
message(STATUS "")
