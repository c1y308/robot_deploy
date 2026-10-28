#pragma once

#include "imu_base/imu_base.hpp"
#include "motor_base/command_types.hpp"
#include "policy/policy_observation_config.hpp"
#include "recorder/inference_recorder_config.hpp"
#include "robot/joint_mapping_config.hpp"
#include "rt_thread_config/thread_runtime.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace inference {

/* 电机硬件与会话配置。mit_kp/mit_kd 为物理电机顺序，由配置加载器转换填充。 */
struct MotorConfig {
    int num_motors = static_cast<int>(motor_base::kMaxMotors);

    int wait_all_motors_timeout_ms = 40000;
    int wait_all_motors_poll_ms    = 100;

    int discrete_command_completion_timeout_ms = 4000;

    bool print_motors_info = true;
    std::vector<int> print_motor_ids = {0, 1, 2, 3, 4, 5,
                                        6, 7, 8, 9, 10, 11};

    motor_base::MotorControlMode control_mode =
        motor_base::MotorControlMode::IMPEDANCE;

    std::array<double, motor_base::kMaxMotors> mit_kp{};   // 物理电机顺序
    std::array<double, motor_base::kMaxMotors> mit_kd{};   // 物理电机顺序
};

struct ImuConfig {
    imu_base::ReaderType type = imu_base::ReaderType::XSENS_MTI_CAN;
    std::string device  = "can0";
    int baudrate        = 921600;
    bool configure_can  = true;
    int can_bitrate     = 250000;

    bool print_imu  = false;
    bool print_ahrs = false;
};

/* 动作后处理：ActionProcessor / RobotInterface::policy_step 消费。
   全部按模型 DOF 顺序。 */
struct ActionConfig {
    std::array<double, policy_observation::kDof> default_joint_pos_rad{};
    std::array<double, policy_observation::kDof> action_scale{};
    /* raw_action * action_scale 之后的偏移量限位: [lower, upper] */
    std::array<std::array<double, 2>, policy_observation::kDof> action_clip{};
    /* nullopt 表示不对策略原始输出做额外裁剪。 */
    std::optional<double> raw_action_clip = 1.0;
};

/* 观测缩放：ObservationBuilder 消费。全部按模型 DOF 顺序。 */
struct ObservationScaleConfig {
    std::array<double, policy_observation::kDof> dof_pos_scale = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };
    std::array<double, policy_observation::kDof> dof_vel_scale = {
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05
    };
    std::array<double, 3> command_scale      = {1.0, 1.0, 1.0};
    std::array<double, 3> body_ang_vel_scale = {0.2, 0.2, 0.2};
};

/* 策略运行时：PolicyRuntime / command worker 消费。 */
struct PolicyRuntimeConfig {
    std::string model_path = "model/policy.onnx";
    double step_dt = 0.02;

    /* 由 YAML 中 observations.gait_phase 是否存在决定；注释掉或缺失时使用 675 维。 */
    struct GaitPhase {
        bool enabled           = false;
        double period          = 0.74;
        double stand_threshold = 0.05;
        double move_threshold  = 0.15;
    } gait;
};

/* 传感器时序守卫：策略入口和命令线程共用电机反馈年龄上限。 */
struct SensorGuardConfig {
    double max_imu_sample_age_s    = 0.050; // IMU 数据最大延迟，超过该值视为过期
    double max_motor_sample_age_s  = 0.020; // 电机状态最大延迟，超过该值视为过期
};

/* 策略帧和 RT 连续命令的独立新鲜度预算。单位 ms。 */
struct SafetyConfig {
    double policy_target_timeout_ms   = 60.0;  // 有效 target 发布后的最大 hold 时间；首帧前从首次推理起计
    double control_command_timeout_ms = 10.0;  // 控制命令的独立新鲜度预算，超过该值视为过期
};

/* 脚踝物理电机的行程限位。
   顺序: left upper, left lower, right upper, right lower。 */
struct AnkleMotorLimitConfig {
    std::array<double, 4> min_rad = {-1.1, -1.1, -1.1, -1.1};
    std::array<double, 4> max_rad = { 1.1,  1.1,  1.1,  1.1};
};

struct AnkleTorqueControlConfig {
    std::array<double, 2> virtual_kp = {187.0, 187.0};    // [pitch, roll]
    std::array<double, 2> virtual_kd = {9.07, 9.07};      // [pitch, roll]

    double filter_cutoff_rad_s   = 100.0;
    double filter_dt_s           = 0.001;
    double motor_rated_torque_nm = 10.5;
    double target_torque_limit_permille = 2000.0;    // 目标扭矩限制，单位为千分比，最大值为 32767
};

struct RuntimeThreadingConfig {
    bool require_host_preflight{false};
    bool require_process_memory_lock{false};

    robot_base::ThreadRuntimeOptions policy_main;
    // 保留独立电机工具的 FIFO80 默认调度，不指定 CPU 集合。
    robot_base::ThreadRuntimeOptions motor_rt{
        {}, robot_base::ThreadSchedulingPolicy::FIFO, 80, 0};
    robot_base::ThreadRuntimeOptions policy_command;
    robot_base::ThreadRuntimeOptions imu_reader;
    robot_base::ThreadRuntimeOptions background;

    int onnx_intra_op_threads{1};
    int onnx_worker_cpu_id{-1};  // 双线程实验模式的额外 worker；单线程时不使用。
};

inline robot_base::ThreadRuntimeOptions make_thread_options(
    std::vector<int> cpu_ids,
    robot_base::ThreadSchedulingPolicy scheduling_policy,
    int priority = 0,
    std::size_t stack_prefault_bytes = 0U)
{
    robot_base::ThreadRuntimeOptions options;
    options.cpu_ids = std::move(cpu_ids);
    options.stack_prefault_bytes = stack_prefault_bytes;
    options.scheduling_policy = scheduling_policy;
    options.priority = priority;
    return options;
}


// 固定 RK3588 部署配置，由调用方显式选择。
inline RuntimeThreadingConfig make_rk3588_runtime_profile()
{
    RuntimeThreadingConfig profile;
    profile.require_host_preflight = true;
    profile.require_process_memory_lock = true;
    using robot_base::ThreadSchedulingPolicy;
    profile.policy_main     = make_thread_options({4}, ThreadSchedulingPolicy::OTHER,
                                                  0, 128U * 1024U);
    profile.motor_rt        = make_thread_options({7}, ThreadSchedulingPolicy::FIFO,
                                                  80, 128U * 1024U);
    profile.policy_command  = make_thread_options({6}, ThreadSchedulingPolicy::FIFO,
                                                  70, 128U * 1024U);
    profile.imu_reader      = make_thread_options({2}, ThreadSchedulingPolicy::OTHER,
                                                  0, 64U * 1024U);
    profile.background      = make_thread_options({0, 1}, ThreadSchedulingPolicy::OTHER);
    profile.onnx_intra_op_threads = 1;
    profile.onnx_worker_cpu_id = 5;
    return profile;
}

struct RobotInterfaceConfig {
    // 电机 运行配置
    MotorConfig motor;

    // IMU 运行配置
    ImuConfig   imu;

    // 传感器数据时效性检查配置
    SensorGuardConfig sensor_guard;

    // 命令时效性检查配置
    SafetyConfig safety;

    // 关节映射配置（物理电机顺序 -> 模型 DOF 顺序）
    JointMappingConfig joint_mapping;

    // 动作后处理配置（包含默认关节位置、动作缩放、动作限位）
    ActionConfig action;

    // 脚踝电机限位配置（包含行程限位、扭矩控制参数）
    AnkleMotorLimitConfig ankle_motor_limits;

    // 脚踝扭矩控制配置（包含虚拟阻尼、滤波器参数、目标扭矩限制）
    AnkleTorqueControlConfig ankle_torque;

    // 观测缩放配置（包含关节位置、关节速度、命令、机体角速度的缩放参数）
    ObservationScaleConfig observation_scales;

    // 策略运行时配置（包含模型路径、步长、705 观测分支的步态参数）
    PolicyRuntimeConfig policy;

    // 日志记录器配置
    InferenceRecorderConfig recorder;

    // 线程运行配置
    RuntimeThreadingConfig runtime;
};

}  // namespace inference
