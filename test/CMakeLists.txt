project(project_robot CXX C)

if(APPLE)
    message(STATUS "Add GTest for Mac ARM processor.")
    message(STATUS "If you haven't installed run:")
    message(STATUS "---> $ brew install googletest")
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/googletest")
elseif(UNIX)
    message(STATUS "We are on a Unix-like system")
elseif(WIN32)
    message(STATUS "We are on Windows")
endif()

set(UT_EXEC "unit_tests")

find_package(GTest REQUIRED)

set (PROJECT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

set(SOURCE_FILES
    ../src/main.c
    ../src/robot.c
    ../src/motor/motor.c
    ../src/imu/imu.c
    ../src/imu/mpu6050.c
    ../src/communication/comm_bus.c
    ../src/communication/i2c.c
)

list(REMOVE_ITEM SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../src/main.c")

set(TEST_SOURCES
    test_main.cpp
)

add_executable(${UT_EXEC} ${TEST_SOURCES} ${SOURCES})

target_link_libraries(${UT_EXEC} GTest::GTest GTest::Main)

# GTest UnitTests
enable_testing()
add_test(UnitTests ${UT_EXEC})

