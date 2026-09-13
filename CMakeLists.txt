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
#   - FreeRTOS-Kernel: Real-time operating system
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

# Sets MCU_*, LIBOPENCM3_*, LINKER_SCRIPT, CPU_FLAGS, FREERTOS_*, SYS_CLOCK_HZ,
# BOARD_SOURCE, BOARD_INCLUDE_DIR, MOTOR_SOURCE and BOARD_DEFAULT_* (see below)
include(${CMAKE_SOURCE_DIR}/cmake/boards/${BOARD}.cmake)

# =============================================================================
# Configuration: prj.conf, board .conf and overlays (see cmake/features.cmake)
# =============================================================================

include(${CMAKE_SOURCE_DIR}/cmake/features.cmake)
robot_load_configuration()

# ---- Values -----------------------------------------------------------------

robot_option(APP_BLINK_ONLY BOOL OFF "Only run the LED heartbeat task (no drivers, all features off)")
robot_option(UART_BAUDRATE INT 921600 "USB console baud rate (telemetry and AT commands)")
robot_option(BT_BAUDRATE INT 115200 "Bluetooth console baud rate")
robot_option(IMU_SAMPLE_RATE_MS INT 10 "IMU and control loop period in ms")
robot_option(FREERTOS_TICK_RATE_HZ INT 1000 "FreeRTOS tick rate in Hz")
robot_option(FREERTOS_TOTAL_HEAP_SIZE INT ${BOARD_DEFAULT_HEAP_SIZE} "FreeRTOS heap size in bytes")
robot_option(ATTITUDE_FILTER STRING complementary "Tilt estimator used by the balance loop"
    CHOICES complementary kalman)

# The filters and PID use IMU_SAMPLE_RATE_MS as dt, so the task period must be exact
math(EXPR IMU_SAMPLE_PERIOD_REMAINDER "(${IMU_SAMPLE_RATE_MS} * ${FREERTOS_TICK_RATE_HZ}) % 1000")
if(NOT IMU_SAMPLE_PERIOD_REMAINDER EQUAL 0)
    message(FATAL_ERROR
        "IMU_SAMPLE_RATE_MS=${IMU_SAMPLE_RATE_MS} is not a whole number of ticks at "
        "FREERTOS_TICK_RATE_HZ=${FREERTOS_TICK_RATE_HZ}: the loop period would differ from dt."
    )
endif()

# ---- Features ---------------------------------------------------------------

robot_feature(CONSOLE_USB ON "AT console on the USB console UART; carries telemetry and fault reports"
    REQUIRES "NOT APP_BLINK_ONLY")
robot_feature(CONSOLE_BT ${BOARD_HAS_BT_UART} "AT console on the Bluetooth module UART"
    REQUIRES "NOT APP_BLINK_ONLY" BOARD_HAS_BT_UART)
robot_define(CONSOLE_ANY CONSOLE_USB OR CONSOLE_BT)

if(CONSOLE_ANY)
    # Shared by both consoles
    set_property(GLOBAL APPEND PROPERTY ROBOT_FEATURE_SOURCES
        ${SRC_DIR}/drivers/uart.c
        ${SRC_DIR}/cmd/at_cmd.c
        ${SRC_DIR}/util/fmt.c
        ${SRC_DIR}/robot/robot_commands.c
        ${SRC_DIR}/app/console.c
    )
    robot_add_module(console_module)
endif()

robot_feature(CONSOLE_ECHO ON "Echo each command line on the USB console" REQUIRES CONSOLE_USB)
robot_feature(TELEMETRY ON "AT+STREAM control loop telemetry on the USB console"
    REQUIRES CONSOLE_USB SOURCES ${SRC_DIR}/telemetry/telemetry.c MODULES telemetry_module)
robot_feature(AT_CMD_PID_TOGGLE ON "AT+PID, AT+PIDON and AT+PIDOFF commands" REQUIRES CONSOLE_ANY)
robot_feature(AT_CMD_HELP OFF "AT+HELP command" REQUIRES CONSOLE_ANY)
robot_feature(AT_CMD_ALL_QUERY OFF "AT+ALL? bulk sensor query" REQUIRES CONSOLE_ANY)
robot_feature(LOGGING OFF "Debug log messages on the USB console (~300 bytes stack)" REQUIRES CONSOLE_USB)
robot_feature(FAULT_VERBOSE ON "Print fault reports on the USB console" REQUIRES CONSOLE_USB)
robot_feature(WATCHDOG ON "Independent watchdog refreshed by the control task" REQUIRES "NOT APP_BLINK_ONLY")
robot_feature(I2C_BUS_RECOVERY ON "Free a stuck I2C slave at start-up" REQUIRES "NOT APP_BLINK_ONLY")
robot_feature(AUTO_ENABLE OFF "Start balancing once per boot after the robot is held upright"
    REQUIRES "NOT APP_BLINK_ONLY")

if(NOT APP_BLINK_ONLY AND NOT CONSOLE_ANY AND NOT AUTO_ENABLE)
    message(FATAL_ERROR
        "CONSOLE_USB and CONSOLE_BT are both off, so nothing could send AT+ENABLE.\n"
        "Enable a console or set AUTO_ENABLE=ON."
    )
endif()

robot_write_config_header(${SRC_DIR}/app_config.h.in ${GENERATED_DIR}/app_config.h)

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
# NOTE: src/rtos_glue/opencm3.c is linked with app (not library) to ensure interrupt
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
    ${SRC_DIR}/rtos_glue/opencm3.c
)

if(NOT APP_BLINK_ONLY)
    list(APPEND APP_SOURCES
        ${SRC_DIR}/interrupts.c
        ${SRC_DIR}/drivers/i2c.c
        ${SRC_DIR}/drivers/pwm.c
        ${SRC_DIR}/filter/kalman.c
        ${SRC_DIR}/filter/complementary.c
        ${SRC_DIR}/imu/imu.c
        ${SRC_DIR}/imu/mpu6050.c
        ${MOTOR_SOURCE}
        ${SRC_DIR}/robot/robot.c
        ${SRC_DIR}/control/pid.c
        ${SRC_DIR}/control/mixer.c
    )
    robot_add_module(imu_module)
    robot_add_module(robot_module)
endif()

# Sources of enabled features
get_property(FEATURE_SOURCES GLOBAL PROPERTY ROBOT_FEATURE_SOURCES)
list(APPEND APP_SOURCES ${FEATURE_SOURCES})

# Enabled modules; the LED heartbeat is always there
robot_add_module(led_module)
robot_write_modules(${SRC_DIR}/app/app_modules.c.in ${GENERATED_DIR}/app_modules.c)
list(APPEND APP_SOURCES ${GENERATED_DIR}/app_modules.c)

# =============================================================================
# Main Executable
# =============================================================================

add_executable(${PROJECT_NAME}.elf ${APP_SOURCES})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
    ${SRC_DIR}
    ${BOARD_INCLUDE_DIR}
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
# API Documentation (Doxygen)
# =============================================================================

find_package(Doxygen OPTIONAL_COMPONENTS dot)

if(DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/docs)
    if(DOXYGEN_DOT_FOUND)
        set(DOXYGEN_HAVE_DOT YES)
    else()
        set(DOXYGEN_HAVE_DOT NO)
    endif()

    configure_file(${CMAKE_SOURCE_DIR}/docs/doxygen/Doxyfile.in
        ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)

    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
        COMMENT "Generating API documentation in ${DOXYGEN_OUTPUT_DIR}/html"
        VERBATIM
    )
else()
    message(STATUS "doxygen not found - 'docs' target will not be available (sudo apt install doxygen graphviz)")
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
message(STATUS "Toolchain:      ${CMAKE_C_COMPILER}")
message(STATUS "libopencm3:     ${LIBOPENCM3_LIB}")
message(STATUS "FreeRTOS:       ${FREERTOS_INCLUDE_DIRS}")
message(STATUS "Linker script:  ${LINKER_SCRIPT}")
message(STATUS "")
