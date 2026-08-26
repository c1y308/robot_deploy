cmake_minimum_required(VERSION 3.8)
project(robot_base CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(robot_base INTERFACE)
target_include_directories(robot_base INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(spsc_latest_value_test
    tests/spsc_latest_value_test.cpp
)
target_link_libraries(spsc_latest_value_test PRIVATE
    robot_base
    pthread
)
