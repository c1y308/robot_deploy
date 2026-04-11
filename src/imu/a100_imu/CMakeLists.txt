cmake_minimum_required(VERSION 3.10)
project(rk3588_imu_reader_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES
    src/main.cpp
    src/serial_port.cpp
    src/imu_parser.cpp
    src/imu_reader.cpp
)

add_executable(imu_reader_cpp ${SOURCES})

target_link_libraries(imu_reader_cpp
    m
    pthread
)

install(TARGETS imu_reader_cpp
    RUNTIME DESTINATION bin
)

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Sources: ${SOURCES}")
