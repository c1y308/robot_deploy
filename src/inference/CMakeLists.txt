cmake_minimum_required(VERSION 3.10)
project(robot_inference_hw_interface CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Export compile commands" FORCE)
enable_testing()

set(ONNXRUNTIME_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime-1.30.0"
    CACHE PATH "Path to the ONNX Runtime Linux AArch64 C++ SDK")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h" OR
   NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
    message(FATAL_ERROR "ONNX Runtime C++ SDK not found at ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT}")
endif()
if(NOT EXISTS "${ONNXRUNTIME_ROOT}/VERSION_NUMBER")
    message(FATAL_ERROR "ONNX Runtime SDK lacks VERSION_NUMBER at ${ONNXRUNTIME_ROOT}")
endif()
file(READ "${ONNXRUNTIME_ROOT}/VERSION_NUMBER" ONNXRUNTIME_VERSION)
string(STRIP "${ONNXRUNTIME_VERSION}" ONNXRUNTIME_VERSION)
if(NOT ONNXRUNTIME_VERSION STREQUAL "1.30.0")
    message(FATAL_ERROR "ONNX Runtime 1.30.0 required, found ${ONNXRUNTIME_VERSION}")
endif()
add_library(ONNXRuntime::onnxruntime SHARED IMPORTED)
set_target_properties(ONNXRuntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}")
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
set(CMAKE_BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")

# 与 ONNX Runtime 及其他系统库使用当前工具链的默认 C++ ABI。
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
target_link_libraries(inference_recorder PUBLIC robot_base pthread)

# deploy.yaml 加载器：解析真实 schema，并校验模型序/电机序边界。
# 只依赖纯类型（MotorControlMode/ReaderType）与 JointMapping，不依赖推理库。
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
    ROBOT_DEPLOY_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/deploy.yaml"
    ROBOT_INFERENCE_ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(deploy_config PUBLIC robot_base yaml-cpp)

add_library(robot_interface STATIC
    src/robot/robot_interface.cpp
    src/robot/policy_command_worker.cpp
    src/robot/action_processor.cpp
    src/robot/observation_builder.cpp
    src/robot/robot_motor_session.cpp
    src/robot/robot_imu_session.cpp
    src/policy/policy_runtime.cpp
)
target_include_directories(robot_interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../motors/src/protocol/ethercat
    ${CMAKE_CURRENT_SOURCE_DIR}/../imu/include
)
target_link_libraries(robot_interface
    PUBLIC
    deploy_config
    inference_recorder
    robot_base
    myactua_ethercat
    imu_a100_core
    imu_xsens_mti_core
    ONNXRuntime::onnxruntime
    pthread
)

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

add_executable(motor_sequence_test
    examples/motor_sequence_test.cpp
)
target_link_libraries(motor_sequence_test PRIVATE robot_interface)

add_executable(policy_test
    examples/policy_test.cpp
)
target_link_libraries(policy_test PRIVATE
    robot_interface
    xbox_control_core
)

add_test(NAME robot_rt_setup_fixture_test COMMAND bash
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deploy/rk3588/test_robot_rt_setup.sh
)
