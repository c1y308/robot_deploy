cmake_minimum_required(VERSION 3.10)
project(robot_inference_hw_interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../motors
                 ${CMAKE_CURRENT_BINARY_DIR}/motors_build)

add_library(imu_a100_core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/src/drivers/serial_port.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/src/protocol/frame/imu_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/src/drivers/imu_reader.cpp
)
target_include_directories(imu_a100_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/src/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/src/protocol/frame
)
target_link_libraries(imu_a100_core PUBLIC m pthread)

add_library(robot_interface STATIC
    src/robot_interface.cpp
)
target_include_directories(robot_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/drivers/myact
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/protocol/ethercat
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/include
)
target_link_libraries(robot_interface
    PUBLIC
    MYACTUA_EtherCat
    imu_a100_core
    pthread
)


add_executable(imu_only_test
    examples/imu_test.cpp
)
target_link_libraries(imu_only_test PRIVATE robot_interface)

add_executable(motors_test
    examples/motors_test.cpp
)
target_link_libraries(motors_test PRIVATE robot_interface)
