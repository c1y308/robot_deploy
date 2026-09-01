#pragma once

#include "imu_base/imu_base.hpp"
#include "motor_base/command_types.hpp"
#include "policy/policy_observation_config.hpp"
#include "recorder/inference_recorder_config.hpp"
#include "robot/joint_mapping_config.hpp"

#include <array>
#include <string>
#include <vector>

namespace inference {

/* 电机硬件与会话配置。mit_kp/mit_kd 为物理电机顺序，由配置加载器转换填充。 */
struct MotorConfig {
    int num_motors;
    std::string ethercat_ifname;

    int wait_all_motors_timeout_ms;
    int wait_all_motors_poll_ms;
    int control_ready_timeout_ms;

    bool print_motors_info;
    std::vector<int> print_motor_ids;

    motor_base::MotorControlMode control_mode;
    std::array<double, motor_base::kMaxMotors> mit_kp;   // 物理电机顺序
    std::array<double, motor_base::kMaxMotors> mit_kd;   // 物理电机顺序
};

struct ImuConfig {
    imu_base::ReaderType type;
    std::string device;
    int baudrate;

    bool print_imu;
    bool print_ahrs;
};

/* 动作后处理：ActionProcessor / RobotInterface::policy_step 消费。
   全部按模型 DOF 顺序。 */
struct ActionConfig {
    std::array<double, policy_observation::kDof> default_joint_pos_rad;
    std::array<double, policy_observation::kDof> action_scale;
    /* raw_action * action_scale 之后的偏移量限位: [lower, upper] */
    std::array<std::array<double, 2>, policy_observation::kDof> action_clip;
    double raw_action_clip;
};

/* 观测缩放：ObservationBuilder 消费。全部按模型 DOF 顺序。 */
struct ObservationScaleConfig {
    std::array<double, policy_observation::kDof> dof_pos_scale;
    std::array<double, policy_observation::kDof> dof_vel_scale;
    std::array<double, 3> command_scale;
    std::array<double, 3> body_ang_vel_scale;
};

/* 策略运行时：PolicyRuntime / command worker 消费。 */
struct PolicyRuntimeConfig {
    std::string model_path;
    double step_dt;
    double target_interpolation_duration_s;

    /* 仅 705 观测分支（ROBOT_POLICY_ENABLE_GAIT_PHASE_OBS=ON）使用；
       675 编译时该段禁止出现在配置文件中，由加载器检查。 */
    struct GaitPhase {
        double period;
        double stand_threshold;
        double move_threshold;
    } gait;
};

/* 传感器时序守卫：RobotInterface::policy_step 消费。 */
struct SensorGuardConfig {
    double max_imu_sample_age_s;
    double max_motor_sample_age_s;
    double max_sensor_state_skew_s;
};

/* 脚踝物理电机的行程限位。
   顺序: left upper, left lower, right upper, right lower。 */
struct AnkleMotorLimitConfig {
    std::array<double, 4> min_rad;
    std::array<double, 4> max_rad;
};

struct AnkleTorqueControlConfig {
    std::array<double, 2> virtual_kp;    // [pitch, roll]
    std::array<double, 2> virtual_kd;    // [pitch, roll]
    double filter_cutoff_rad_s;
    double filter_dt_s;
    double motor_rated_torque_nm;
    double target_torque_limit_permille;    // 目标扭矩限制，单位为千分比，最大值为 32767
};

struct RobotInterfaceConfig {
    MotorConfig motor;
    ImuConfig   imu;
    JointMappingConfig joint_mapping;
    ActionConfig action;
    ObservationScaleConfig observation_scales;
    PolicyRuntimeConfig policy;
    SensorGuardConfig sensor_guard;
    AnkleMotorLimitConfig ankle_motor_limits;
    AnkleTorqueControlConfig ankle_torque;
    InferenceRecorderConfig recorder;
};

}  // namespace inference
