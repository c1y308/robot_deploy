cmake_minimum_required(VERSION 3.8)
project(robot_base CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(robot_base INTERFACE)
target_include_directories(robot_base INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(robot_base INTERFACE rt)

add_executable(spsc_latest_value_test
    tests/spsc_latest_value_test.cpp
)
target_link_libraries(spsc_latest_value_test PRIVATE
    robot_base
    pthread
)

add_executable(ring_buffer_test
    tests/ring_buffer_test.cpp
)
target_link_libraries(ring_buffer_test PRIVATE
    robot_base
)
