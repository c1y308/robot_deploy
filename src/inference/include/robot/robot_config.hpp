#pragma once

#include "imu_base/imu_base.hpp"
#include "motor_base/command_types.hpp"
#include "recorder/inference_recorder_config.hpp"
#include "robot/joint_mapping.hpp"
#include "thread/thread_runtime.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace inference {

// 策略控制的模型关节维度，按模型关节顺序。
inline constexpr std::size_t kDof = 12;
static_assert(kDof == motor_base::kMotorNum,
              "the current robot mapping requires one motor per model DOF");

/* 动作后处理：ActionProcessor / RobotInterface::policy_step 消费。
   全部按模型 DOF 顺序。 */
struct ActionConfig {
    std::array<double, kDof> action_scale{};
    /* raw_action * action_scale 之后的偏移量限位: [lower, upper] */
    std::array<std::array<double, 2>, kDof> action_clip{};
    /* nullopt 表示不对策略原始输出做额外裁剪。 */
    std::optional<double> raw_action_clip = 1.0;
};

struct AnkleControlConfig {
    // 物理电机位置限位：[左上、左下、右上、右下]。
    std::array<double, 4> motor_pos_min_rad = {-1.1, -1.1, -1.1, -1.1};
    std::array<double, 4> motor_pos_max_rad = { 1.1,  1.1,  1.1,  1.1};

    // 虚拟关节增益：[pitch, roll]，左右脚共用。
    std::array<double, 2> virtual_kp = {187.0, 187.0};
    std::array<double, 2> virtual_kd = {9.07, 9.07};

    // 扭矩低通滤波；filter_dt_s 同时作为命令线程循环周期。
    double filter_cutoff_rad_s   = 100.0;
    double filter_dt_s           = 0.001;

    // 电机额定扭矩与输出扭矩限幅。
    double motor_rated_torque_nm = 10.5;
    double target_torque_limit_permille = 2000.0;    // 目标扭矩限制，单位为千分比，最大值为 32767
};

/* 电机硬件与会话配置。mit_kp/mit_kd 为物理电机顺序，由配置加载器转换填充。 */
struct MotorConfig {
    int num_motors = static_cast<int>(motor_base::kMotorNum);

    int wait_all_motors_timeout_ms = 40000;
    int wait_all_motors_poll_ms    = 100;

    int discrete_command_completion_timeout_ms = 4000;

    bool print_motors_info = true;
    std::vector<int> print_motor_ids = {0, 1, 2, 3, 4, 5,
                                        6, 7, 8, 9, 10, 11};

    motor_base::MotorControlMode control_mode = motor_base::MotorControlMode::IMPEDANCE;

    std::array<double, motor_base::kMotorNum> mit_kp{};   // 物理电机顺序
    std::array<double, motor_base::kMotorNum> mit_kd{};   // 物理电机顺序
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

/* 观测缩放：ObservationBuilder 消费。全部按模型 DOF 顺序。 */
struct ObservationScaleConfig {
    std::array<double, kDof> dof_pos_scale = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };
    std::array<double, kDof> dof_vel_scale = {
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05
    };
    std::array<double, 3> command_scale      = {1.0, 1.0, 1.0};
    std::array<double, 3> body_ang_vel_scale = {0.2, 0.2, 0.2};
    std::array<double, 3> projected_gravity_scale = {1.0, 1.0, 1.0};
    std::array<double, kDof> last_action_scale = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    };
    std::array<double, 2> gait_phase_scale = {1.0, 1.0};
};

/* YAML 中的必填速度范围；加载器确保三项均已配置。 */
struct VelocityCommandRangeConfig {
    std::optional<std::array<double, 2>> lin_vel_x;
    std::optional<std::array<double, 2>> lin_vel_y;
    std::optional<std::array<double, 2>> ang_vel_z;
};

/* 策略运行时：PolicyRuntime / command worker 消费。 */
struct PolicyRuntimeConfig {
    std::string model_path = "model/policy.onnx";
    double step_dt = 0.02;

    /* 由 YAML 中 observations.gait_phase 是否存在决定；注释掉或缺失时使用 675 维。 */
    struct GaitConfig {
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
    // 机器人默认姿态，按模型关节顺序；用于动作基准、观测参考和启动复位。
    std::array<double, kDof> default_joint_pos_rad{};

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

    // 动作后处理配置（包含动作缩放、动作限位）
    ActionConfig action;

    // 脚踝控制：电机位置限位、虚拟关节增益、扭矩滤波与限幅。
    AnkleControlConfig ankle;

    // 各观测项的缩放配置
    ObservationScaleConfig observation_scales;

    // YAML 提供的目标速度范围
    VelocityCommandRangeConfig command_ranges;

    // 策略运行时配置（包含模型路径、步长、705 观测分支的步态参数）
    PolicyRuntimeConfig policy;

    // 日志记录器配置
    InferenceRecorderConfig recorder;

    // 线程运行配置
    RuntimeThreadingConfig runtime;
};

}  // namespace inference
