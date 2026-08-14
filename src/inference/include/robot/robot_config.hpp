#pragma once

#include "motor_base/command_types.hpp"
#include "recorder/inference_record.hpp"
#include "robot/joint_mapping.hpp"

#include <array>
#include <limits>
#include <string>
#include <vector>

namespace inference {

struct MotorConfig {
    int num_motors = 12;
    std::string ethercat_ifname = "enp8s0";

    int wait_all_motors_timeout_ms = 20000;
    int wait_all_motors_poll_ms    = 100;

    bool print_motors_info = false;
    std::vector<int> print_motor_ids = {-1};

    motor_base::MotorControlMode control_mode = motor_base::MotorControlMode::IMPEDANCE;
    std::vector<double> mit_kp;
    std::vector<double> mit_kd;
};

struct ImuConfig {
    std::string device = "/dev/ttyUSB0";
    int baudrate       = 921600;

    bool print_imu     = false;
    bool print_ahrs    = false;
};

struct PolicyConfig {
    std::string model_path;

    std::vector<double> joint_min_rad;
    std::vector<double> joint_max_rad;
    std::vector<double> stand_pose_rad = {};

    std::vector<std::array<double, 2>> action_clip;
    std::vector<double> action_scale;
    std::vector<double> dof_pos_scale;
    std::vector<double> dof_vel_scale;

    std::array<double, 3> command_scale = {
        0.1,
        0.1,
        0.1
    };
    std::array<double, 3> body_ang_vel_scale = {
        0.2,
        0.2,
        0.2
    };

    double raw_action_clip = 1.0;
    double step_dt = 0.02;
    double gait_phase_period = 0.74;
    double gait_phase_stand_threshold = 0.05;
    double gait_phase_move_threshold = 0.15;
};

struct AnkleTorqueControlConfig {
    std::array<double, 2> virtual_kp = {180.0, 180.0};  // [pitch, roll]
    std::array<double, 2> virtual_kd = {10.0, 10.0};    // [pitch, roll]
    double filter_cutoff_rad_s = 100.0;
    double filter_dt_s = 0.001;
    double motor_rated_torque_nm = 10.5;
    double target_torque_limit_permille = 800.0;    // 目标扭矩限制，单位为千分比，最大值为 32767
};

struct RobotInterfaceConfig {
    MotorConfig motor;
    ImuConfig   imu;
    JointMappingConfig joint_mapping;
    PolicyConfig policy;
    AnkleTorqueControlConfig ankle_torque;
    InferenceRecorderConfig recorder;
};

}  // namespace inference
