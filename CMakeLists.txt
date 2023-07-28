cmake_minimum_required(VERSION 3.10)
project(project_robot C CXX)

# Add the source files for the main application
set(SOURCE_FILES
    src/main.c
    src/robot.c
    src/motor.c
    src/imu/imu.c
    src/imu/mpu6050.c
)

# Add the source files for the test cases
set(TEST_SOURCE_FILES
    test/test_main.c
    test/test_robot.c
    test/test_motor.c
)

# Include FreeRTOS configuration
# include_directories(${CMAKE_SOURCE_DIR}/build_stm32)

# Set the output binary name and path for STM32 target
set(STM32_EXECUTABLE_NAME "robot_firmware.elf")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/build")

# Set the output binary name and path for PC target
set(PC_EXECUTABLE_NAME "robot_simulator")

# Check the target architecture and set the compiler flags accordingly
# if (CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
#     message(STATUS "Cross-compiling for STM32 target.")
#     set(STM32_TARGET TRUE)

#     # Set the compiler flags and options for the STM32 microcontroller
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs")

#     # Add the STM32 linker script and startup assembly code
#     # set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/build_stm32/stm32.ld")
#     # set(STARTUP_CODE "${CMAKE_SOURCE_DIR}/build_stm32/startup_stm32.s")
# else()
    message(STATUS "Building for PC target.")
    set(STM32_TARGET FALSE)
# endif()

# Create the main application target (STM32)
if (STM32_TARGET)
    add_executable(${STM32_EXECUTABLE_NAME} ${SOURCE_FILES} ${STARTUP_CODE})
    target_link_options(${STM32_EXECUTABLE_NAME} PRIVATE "LINKER:${LINKER_SCRIPT}")
    target_compile_features(${STM32_EXECUTABLE_NAME} PRIVATE c_std_11)
endif()

# Create the PC executable target
add_executable(${PC_EXECUTABLE_NAME} ${SOURCE_FILES})
target_compile_features(${PC_EXECUTABLE_NAME} PRIVATE c_std_11)

if(DEFINED ALTERNATE_MAIN_APP_DIR)
    # Add test target
    add_executable(tests ${TEST_SOURCE_FILES} ${SOURCE_FILES})
    target_include_directories(tests PRIVATE src test/unity)

    # Set the C++ standard to C++11 for the test target (Google Test and Google Mock require C++11)
    set_target_properties(tests PROPERTIES CXX_STANDARD 11)
else()
endif()
# Add Google Test and Google Mock libraries for the test target
# Assuming you have downloaded and built Google Test and Google Mock libraries separately
# and they are available in the directory specified by GTEST_ROOT and GMOCK_ROOT respectively
# set(GTEST_ROOT "/path/to/google_test")
# set(GMOCK_ROOT "/path/to/google_mock")
# target_include_directories(tests PRIVATE ${GTEST_ROOT}/include ${GMOCK_ROOT}/include)
# target_link_directories(tests PRIVATE ${GTEST_ROOT}/lib ${GMOCK_ROOT}/lib)
# target_link_libraries(tests PRIVATE gtest gmock gtest_main)

# Set the C standard to C11 for all targets

# Add any additional target-specific configurations here

# Add any target-specific libraries here

# For example, if you want to use FreeRTOS library for STM32 target:
# if (STM32_TARGET)
#     target_link_libraries(${STM32_EXECUTABLE_NAME} PRIVATE freertos)
# endif()