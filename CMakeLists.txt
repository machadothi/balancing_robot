cmake_minimum_required(VERSION 3.10)

project(balancing_robot)
set(APP_NAME ${PROJECT_NAME})

# add source folder!
add_subdirectory(src)

# GTest UnitTests
add_subdirectory(test EXCLUDE_FROM_ALL)