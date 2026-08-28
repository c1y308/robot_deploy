cmake_minimum_required(VERSION 3.10)
project(rk3588_imu_reader_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(imu_a100_core STATIC
    src/drivers/serial_port.cpp
    src/protocol/a100/imu_parser.cpp
    src/drivers/a100/a100_reader.cpp
)

target_include_directories(imu_a100_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(imu_a100_core
    m
    pthread
)

add_library(imu_xsens_mti_core STATIC
    src/drivers/socket_can_port.cpp
    src/protocol/xsens_mti/can_parser.cpp
    src/drivers/xsens_mti/xsens_reader.cpp
)

target_include_directories(imu_xsens_mti_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(imu_xsens_mti_core
    m
    pthread
)

add_executable(a100_test
    examples/a100_test.cpp
)
target_link_libraries(a100_test PRIVATE imu_a100_core)

add_executable(xsens_mti_can_test
    examples/xsens_mti_can_test.cpp
)
target_link_libraries(xsens_mti_can_test PRIVATE imu_xsens_mti_core)

add_executable(xsens_mti_can_parser_test
    tests/xsens_mti_can_parser_test.cpp
)
target_link_libraries(xsens_mti_can_parser_test PRIVATE imu_xsens_mti_core)
add_test(NAME xsens_mti_can_parser_test COMMAND xsens_mti_can_parser_test)

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
