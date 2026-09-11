cmake_minimum_required(VERSION 3.10)
project(robot_inference_hw_interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)
enable_testing()

option(ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS
       "Include gait phase in policy observation history, using 705 input dimensions instead of 675"
       ON)

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

# 不用系统包（新 ABI），因为 torch 强制 _GLIBCXX_USE_CXX11_ABI=0，
# 同一进程内必须统一 ABI；此处以源码随工程编译旧 ABI 版本。
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp
                 ${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_build)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../motors
                 ${CMAKE_CURRENT_BINARY_DIR}/motors_build)

if(NOT TARGET imu_a100_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../imu
                     ${CMAKE_CURRENT_BINARY_DIR}/imu_build)
endif()

add_library(inference_recorder STATIC
    src/recorder/inference_recorder.cpp
)
target_include_directories(inference_recorder PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(inference_recorder PUBLIC
    ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=$<BOOL:${ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS}>
)
target_link_libraries(inference_recorder PUBLIC robot_base pthread)

# deploy.yaml 加载器：解析真实 schema，并校验模型序/电机序边界。
# 只依赖纯类型（MotorControlMode/ReaderType）与 JointMapping，不依赖 torch。
add_library(deploy_config STATIC
    src/config/deploy_config.cpp
    src/robot/joint_mapping.cpp
)
target_include_directories(deploy_config PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
target_compile_definitions(deploy_config PUBLIC
    ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=$<BOOL:${ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS}>
    ROBOT_DEPLOY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/deploy.yaml"
    ROBOT_INFERENCE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(deploy_config PUBLIC robot_base yaml-cpp)

add_library(robot_interface STATIC
    src/robot/robot_interface.cpp
    src/robot/action_processor.cpp
    src/robot/observation_builder.cpp
    src/robot/robot_motor_session.cpp
    src/robot/robot_imu_session.cpp
    src/robot/rk3588_host_preflight.cpp
    src/policy/policy_runtime.cpp
)
target_include_directories(robot_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/protocol/ethercat
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
target_compile_definitions(robot_interface PUBLIC
    ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=$<BOOL:${ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS}>
)
target_link_libraries(robot_interface
    PUBLIC
    deploy_config
    inference_recorder
    robot_base
    myactua_ethercat
    imu_a100_core
    imu_xsens_mti_core
    ${TORCH_LIBRARIES}
    dl
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
        ${CMAKE_CURRENT_SOURCE_DIR}/../base/include
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

add_executable(inference_recorder_csv_test
    tests/inference_recorder_csv_test.cpp
)
target_link_libraries(inference_recorder_csv_test PRIVATE inference_recorder)
add_test(NAME inference_recorder_csv_test COMMAND inference_recorder_csv_test)

add_executable(action_processor_ankle_ik_test
    tests/action_processor_ankle_ik_test.cpp
    src/robot/action_processor.cpp
)
target_include_directories(action_processor_ankle_ik_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
target_link_libraries(action_processor_ankle_ik_test PRIVATE deploy_config robot_base)
add_test(NAME action_processor_ankle_ik_test COMMAND action_processor_ankle_ik_test)

add_executable(ankle_inverse_kinematics_test
    tests/ankle_inverse_kinematics_test.cpp
    src/robot/robot_motor_session.cpp
)
target_include_directories(ankle_inverse_kinematics_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/protocol/ethercat
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
target_link_libraries(ankle_inverse_kinematics_test PRIVATE
    myactua_ethercat
    robot_base
)

add_executable(ankle_motor_jacobian_test
    tests/ankle_motor_jacobian_test.cpp
)
target_include_directories(ankle_motor_jacobian_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME ankle_motor_jacobian_test COMMAND ankle_motor_jacobian_test)

add_executable(robot_imu_session_config_test
    tests/robot_imu_session_config_test.cpp
)
target_include_directories(robot_imu_session_config_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../base/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
add_test(NAME robot_imu_session_config_test
         COMMAND robot_imu_session_config_test)

add_executable(rk3588_runtime_profile_test
    tests/rk3588_runtime_profile_test.cpp
)
target_include_directories(rk3588_runtime_profile_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rk3588_runtime_profile_test PRIVATE deploy_config robot_base)
add_test(NAME rk3588_runtime_profile_test COMMAND rk3588_runtime_profile_test)

add_executable(rk3588_host_preflight_test
    tests/rk3588_host_preflight_test.cpp
    src/robot/rk3588_host_preflight.cpp
)
target_include_directories(rk3588_host_preflight_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
add_test(NAME rk3588_host_preflight_test COMMAND rk3588_host_preflight_test)

add_executable(policy_runtime_threading_test
    tests/policy_runtime_threading_test.cpp
)
target_compile_definitions(policy_runtime_threading_test PRIVATE
    ROBOT_TEST_POLICY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/policy.pt"
)
target_link_libraries(policy_runtime_threading_test PRIVATE robot_interface)
add_test(NAME policy_runtime_threading_test COMMAND policy_runtime_threading_test)
