cmake_minimum_required(VERSION 3.10)
project(xbox_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)

set(INFERENCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../inference)
set(LOCAL_LIBTORCH_DIR ${INFERENCE_DIR}/third_party/libtorch)
set(LOCAL_PYTORCH_DIR ${INFERENCE_DIR}/third_party/python_torch/torch)
set(TORCH_PREFIX_HINTS)

if(EXISTS "${LOCAL_LIBTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake")
    list(APPEND TORCH_PREFIX_HINTS "${LOCAL_LIBTORCH_DIR}")
endif()

if(EXISTS "${LOCAL_PYTORCH_DIR}/share/cmake/Torch/TorchConfig.cmake")
    list(APPEND TORCH_PREFIX_HINTS "${LOCAL_PYTORCH_DIR}/share/cmake")
endif()

find_program(PYTHON3_EXECUTABLE python3)
if(PYTHON3_EXECUTABLE)
    execute_process(
        COMMAND "${PYTHON3_EXECUTABLE}" -c "import torch; print(torch.utils.cmake_prefix_path, end='')"
        RESULT_VARIABLE PYTORCH_PREFIX_RESULT
        OUTPUT_VARIABLE PYTORCH_CMAKE_PREFIX
        ERROR_QUIET
    )
    if(PYTORCH_PREFIX_RESULT EQUAL 0 AND EXISTS "${PYTORCH_CMAKE_PREFIX}/Torch/TorchConfig.cmake")
        list(APPEND TORCH_PREFIX_HINTS "${PYTORCH_CMAKE_PREFIX}")
    endif()
endif()

if(TORCH_PREFIX_HINTS)
    set(CMAKE_PREFIX_PATH ${TORCH_PREFIX_HINTS} ${CMAKE_PREFIX_PATH})
endif()

find_package(Torch QUIET)
if(NOT Torch_FOUND)
    message(FATAL_ERROR "LibTorch/PyTorch C++ runtime not found. Configure inference dependencies before building xbox_control.")
endif()

add_library(xbox_control_core STATIC
    src/xbox_controller.cpp
)
target_include_directories(xbox_control_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(xbox_input_test
    example/xbox_input_test.cpp
)
target_link_libraries(xbox_input_test PRIVATE
    xbox_control_core
)

if(NOT TARGET robot_interface)
    add_subdirectory(${INFERENCE_DIR}
                     ${CMAKE_CURRENT_BINARY_DIR}/inference_build)
endif()

add_executable(xbox_policy_control
    src/xbox_policy_control.cpp
)
target_link_libraries(xbox_policy_control PRIVATE
    xbox_control_core
    robot_interface
)
