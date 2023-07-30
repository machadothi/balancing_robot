project(robot)
set(APP_NAME ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c11 -Wall -Wextra")

set(SOURCE_FILES
    main.c
    robot.c
    motor/motor.c
    imu/imu.c
    imu/mpu6050.c
    communication/comm_bus.c
    communication/i2c.c
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} )
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})