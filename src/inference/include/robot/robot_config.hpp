#pragma once

#include "imu_base/imu_base.hpp"
#include "motor_base/command_types.hpp"
#include "policy/policy_observation_config.hpp"
#include "recorder/inference_recorder_config.hpp"
#include "robot/joint_mapping_config.hpp"
#include "tool/thread_runtime.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace inference {

/* 电机硬件与会话配置。mit_kp/mit_kd 为物理电机顺序，由配置加载器转换填充。 */
struct MotorConfig {
    int num_motors = static_cast<int>(motor_base::kMaxMotors);
    std::string ethercat_ifname = "enp8s0";

    int wait_all_motors_timeout_ms = 20000;
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
    std::string model_path = "model/policy.pt";
    double step_dt = 0.02;

    /* gait_phase 存在于 deploy.yaml 时使用 705 维观测，否则使用 675 维观测。 */
    struct GaitPhase {
        bool enabled           = false;
        double period          = 0.74;
        double stand_threshold = 0.05;
        double move_threshold  = 0.15;
    } gait;
};

/* 传感器时序守卫：RobotInterface::policy_step 消费。 */
struct SensorGuardConfig {
    double max_imu_sample_age_s    = 0.050; // IMU 数据最大延迟，超过该值视为过期
    double max_motor_sample_age_s  = 0.050; // 电机状态最大延迟，超过该值视为过期
    double max_sensor_state_skew_s = 0.030; // IMU 与电机状态的时间戳差值最大允许值，超过该值视为不同步
};

/* 策略帧和 RT 连续命令的独立新鲜度预算。单位 ms。 */
struct SafetyConfig {
    double policy_target_timeout_ms   = 60.0;  // 最老的数据时刻 + 此值 之后视为过期（防止推理超时）
    double control_command_timeout_ms = 10.0;  // 控制命令的独立新鲜度预算，超过该值视为过期
};

/* 脚踝物理电机的行程限位。
   顺序: left upper, left lower, right upper, right lower。 */
struct AnkleMotorLimitConfig {
    std::array<double, 4> min_rad = {-1.1, -1.1, -1.1, -1.1};
    std::array<double, 4> max_rad = {1.1, 1.1, 1.1, 1.1};
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
    bool enabled{false};
    bool require_host_preflight{false};

    robot_base::ThreadRuntimeOptions policy_main;
    robot_base::ThreadRuntimeOptions motor_rt;
    robot_base::ThreadRuntimeOptions policy_command;
    robot_base::ThreadRuntimeOptions imu_reader;
    robot_base::ThreadRuntimeOptions background;

    int torch_intra_op_threads{0};
    int torch_inter_op_threads{0};
    int openblas_threads{0};
};

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
