cmake_minimum_required(VERSION 3.10)
project(rk3588_imu_reader_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SOURCES
    examples/main.cpp
    src/drivers/serial_port.cpp
    src/protocol/frame/imu_parser.cpp
    src/drivers/imu_reader.cpp
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../motors
                 ${CMAKE_CURRENT_BINARY_DIR}/motors_build)

add_executable(imu_reader_cpp ${SOURCES})

target_include_directories(imu_reader_cpp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol/frame
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motors/src/drivers/myact
    ${CMAKE_CURRENT_SOURCE_DIR}/../../motors/src/protocol/ethercat
)

target_link_libraries(imu_reader_cpp
    MYACTUA_EtherCat
    m
    pthread
)

install(TARGETS imu_reader_cpp
    RUNTIME DESTINATION bin
)

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Sources: ${SOURCES}")
