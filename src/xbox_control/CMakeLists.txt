cmake_minimum_required(VERSION 3.10)
project(xbox_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)
enable_testing()

add_library(xbox_control_core STATIC
    src/xbox_controller.cpp
)
target_include_directories(xbox_control_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../base/include
)
target_link_libraries(xbox_control_core PUBLIC pthread)

add_executable(xbox_input_test
    example/xbox_input_test.cpp
)
target_link_libraries(xbox_input_test PRIVATE
    xbox_control_core
)

add_executable(xbox_controller_mapping_test
    tests/xbox_controller_mapping_test.cpp
)
target_link_libraries(xbox_controller_mapping_test PRIVATE
    xbox_control_core
)
add_test(NAME xbox_controller_mapping_test COMMAND xbox_controller_mapping_test)
