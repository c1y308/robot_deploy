cmake_minimum_required(VERSION 3.10)
project(robot_inference_hw_interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)

option(ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS
       "Include gait phase in policy observation history, using 235 input dimensions instead of 225"
       OFF)

set(LOCAL_LIBTORCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch)
set(LOCAL_PYTORCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/python_torch/torch)
set(LOCAL_PYTORCH_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/python_torch/torch.libs)
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
    message(FATAL_ERROR "LibTorch/PyTorch C++ runtime not found. Put the CPU LibTorch SDK at src/inference/third_party/libtorch, install Python torch, install torch into src/inference/third_party/python_torch, or configure with -DCMAKE_PREFIX_PATH=/path/to/libtorch.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

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
    src/async_csv_logger.cpp
    src/robot_interface.cpp
    src/torch_policy_runner.cpp
)
target_include_directories(robot_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/drivers/myact
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/protocol/ethercat
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/a100/include
)
target_compile_definitions(robot_interface PUBLIC
    ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=$<BOOL:${ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS}>
)
target_link_libraries(robot_interface
    PUBLIC
    MYACTUA_EtherCat
    imu_a100_core
    ${TORCH_LIBRARIES}
    pthread
)
if(EXISTS "${LOCAL_PYTORCH_LIBS_DIR}")
    target_link_libraries(robot_interface
        PUBLIC
        "-Wl,-rpath-link,${LOCAL_PYTORCH_LIBS_DIR}"
        "-Wl,-rpath,${LOCAL_PYTORCH_LIBS_DIR}"
    )
endif()

if(NOT TARGET xbox_control_core)
    add_library(xbox_control_core STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../xbox_control/src/xbox_controller.cpp
    )
    target_include_directories(xbox_control_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../xbox_control/include
    )
    target_link_libraries(xbox_control_core PUBLIC pthread)
endif()

add_executable(imu_only_test
    examples/imu_test.cpp
)
target_link_libraries(imu_only_test PRIVATE robot_interface)

add_executable(motors_test
    examples/motors_test.cpp
)
target_link_libraries(motors_test PRIVATE robot_interface)

add_executable(policy_test
    examples/policy_test.cpp
)
target_link_libraries(policy_test PRIVATE
    robot_interface
    xbox_control_core
)
