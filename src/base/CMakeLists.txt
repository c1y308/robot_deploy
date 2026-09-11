cmake_minimum_required(VERSION 3.8)
project(robot_base CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

add_library(robot_base INTERFACE)
target_include_directories(robot_base INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(robot_base INTERFACE rt pthread)

add_executable(spsc_latest_channel_test
    tests/spsc_latest_channel_test.cpp
)
target_link_libraries(spsc_latest_channel_test PRIVATE
    robot_base
    pthread
)
add_test(NAME spsc_latest_channel_test COMMAND spsc_latest_channel_test)

add_executable(ring_buffer_test
    tests/ring_buffer_test.cpp
)
target_link_libraries(ring_buffer_test PRIVATE
    robot_base
)
add_test(NAME ring_buffer_test COMMAND ring_buffer_test)

add_executable(thread_runtime_test
    tests/thread_runtime_test.cpp
)
target_link_libraries(thread_runtime_test PRIVATE
    robot_base
    pthread
)
add_test(NAME thread_runtime_test COMMAND thread_runtime_test)
