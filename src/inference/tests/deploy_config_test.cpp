#include "config/deploy_config.hpp"
#include "policy/policy_observation_config.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

void expect_near(double actual,
                 double expected,
                 const std::string& message)
{
    if (std::abs(actual - expected) > 1e-12) {
        std::cerr << "FAIL: " << message
                  << " expected=" << expected
                  << " actual=" << actual << "\n";
        std::exit(1);
    }
}

std::filesystem::path unique_temp_path(const std::string& suffix)
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("deploy_config_test_" + std::to_string(now) + "_" + suffix);
}

bool load_yaml_string(const std::string& yaml_text,
                      inference::RobotInterfaceConfig& config,
                      std::string& error)
{
    const std::filesystem::path path = unique_temp_path("case.yaml");
    {
        std::ofstream file(path);
        if (!file) {
            error = "failed to write temp yaml: " + path.string();
            return false;
        }
        file << yaml_text;
    }
    const bool ok = inference::load_deploy_config(path.string(), config, error);
    std::filesystem::remove(path);
    return ok;
}

std::string read_deploy_yaml()
{
    std::ifstream file(ROBOT_DEPLOY_CONFIG_PATH);
    expect(file.good(),
           std::string("deploy.yaml is not readable: ") + ROBOT_DEPLOY_CONFIG_PATH);
    std::ostringstream stream;
    stream << file.rdbuf();
    return stream.str();
}

std::string replace_first(std::string text,
                          const std::string& from,
                          const std::string& to)
{
    const std::size_t pos = text.find(from);
    expect(pos != std::string::npos,
           "replacement source not found in deploy.yaml: " + from);
    text.replace(pos, from.size(), to);
    return text;
}

template <std::size_t N>
void expect_array_near(const std::array<double, N>& actual,
                       const std::array<double, N>& expected,
                       const std::string& message)
{
    for (std::size_t i = 0; i < N; ++i) {
        expect_near(actual[i], expected[i], message + "[" + std::to_string(i) + "]");
    }
}

void test_load_real_deploy_yaml()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    expect(inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, cfg, error),
           "load_deploy_config failed: " + error);

    expect(cfg.motor.ethercat_ifname == "enp8s0", "default ethercat_ifname");
    expect(cfg.imu.type == imu_base::ReaderType::XSENS_MTI_CAN, "default imu.type");
    expect(cfg.imu.device == "can0", "default imu.device");
    expect(cfg.imu.baudrate == 921600, "default imu.baudrate");
    expect(cfg.imu.configure_can, "default imu.configure_can");
    expect(cfg.imu.can_bitrate == 250000, "default imu.can_bitrate");
    expect(!cfg.imu.print_imu && !cfg.imu.print_ahrs, "default imu print flags");

    const std::vector<int> expected_map = {0, 6, 1, 7, 2, 8, 3, 9};
    expect(cfg.joint_mapping.model_to_motor_index == expected_map,
           "joint_mapping.model_to_motor_index");
    const std::array<int, 12> expected_direction = {
        -1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };
    expect(cfg.joint_mapping.motor_to_model_direction == expected_direction,
           "joint_mapping.motor_to_model_direction defaults");
    expect(cfg.joint_mapping.left_ankle_parallel.model_pitch_dof == 8 &&
           cfg.joint_mapping.left_ankle_parallel.model_roll_dof == 10 &&
           cfg.joint_mapping.left_ankle_parallel.upper_motor_index == 4 &&
           cfg.joint_mapping.left_ankle_parallel.lower_motor_index == 5,
           "default left_ankle_parallel");
    expect(cfg.joint_mapping.right_ankle_parallel.model_pitch_dof == 9 &&
           cfg.joint_mapping.right_ankle_parallel.model_roll_dof == 11 &&
           cfg.joint_mapping.right_ankle_parallel.upper_motor_index == 10 &&
           cfg.joint_mapping.right_ankle_parallel.lower_motor_index == 11,
           "default right_ankle_parallel");

    expect(cfg.motor.num_motors == static_cast<int>(motor_base::kMaxMotors),
           "default motor.num_motors");
    expect(cfg.motor.control_mode == motor_base::MotorControlMode::IMPEDANCE,
           "default motor.control_mode");
    const std::array<double, 12> expected_kp = {
        180.0, 180.0, 180.0, 180.0, 187.0, 187.0,
        180.0, 180.0, 180.0, 180.0, 187.0, 187.0
    };
    const std::array<double, 12> expected_kd = {
        10.0, 10.0, 10.0, 10.0, 9.07, 9.07,
        10.0, 10.0, 10.0, 10.0, 9.07, 9.07
    };
    expect_array_near(cfg.motor.mit_kp, expected_kp, "motor.mit_kp");
    expect_array_near(cfg.motor.mit_kd, expected_kd, "motor.mit_kd");
    expect(cfg.motor.wait_all_motors_timeout_ms == 20000,
           "default wait_all_motors_timeout_ms");
    expect(cfg.motor.wait_all_motors_poll_ms == 100,
           "default wait_all_motors_poll_ms");
    expect(cfg.motor.control_ready_timeout_ms == 4000,
           "default control_ready_timeout_ms");
    expect(cfg.motor.print_motors_info, "default motor.print_motors_info");
    expect(cfg.motor.print_motor_ids.size() == 12, "default print_motor_ids size");
    for (std::size_t i = 0; i < cfg.motor.print_motor_ids.size(); ++i) {
        expect(cfg.motor.print_motor_ids[i] == static_cast<int>(i),
               "default print_motor_ids value");
    }

    expect(cfg.policy.model_path ==
               std::string(ROBOT_INFERENCE_ROOT_DIR) + "/model/policy.pt",
           "default policy.model_path resolved against ROBOT_INFERENCE_ROOT_DIR");
    expect_near(cfg.policy.step_dt, 0.02, "policy.step_dt");
    expect_near(cfg.policy.gait.period, 0.74, "default gait.period");
    expect_near(cfg.policy.gait.stand_threshold, 0.05,
                "default gait.stand_threshold");
    expect_near(cfg.policy.gait.move_threshold, 0.15,
                "default gait.move_threshold");

    const std::array<double, 12> expected_default_pos = {
        0.0, 0.0, -0.2, -0.2, 0.0, 0.0,
        0.2, 0.2, -0.05, -0.05, 0.0, 0.0
    };
    const std::array<double, 12> expected_scale = {
        0.1, 0.1, 0.45, 0.45, 0.08, 0.08,
        0.9, 0.9, 0.35, 0.35, 0.08, 0.08
    };
    const std::array<std::array<double, 2>, 12> expected_clip = {{
        {-0.12, 0.12}, {-0.12, 0.12}, {-0.55, 0.4}, {-0.55, 0.4},
        {-0.12, 0.12}, {-0.12, 0.12}, {-0.3, 1.1}, {-0.3, 1.1},
        {-0.45, 0.4},  {-0.45, 0.4},  {-0.1, 0.1}, {-0.1, 0.1}
    }};
    expect_array_near(cfg.action.default_joint_pos_rad, expected_default_pos,
                      "action.default_joint_pos_rad");
    expect_array_near(cfg.action.action_scale, expected_scale,
                      "action.action_scale");
    expect(cfg.action.action_clip == expected_clip, "action.action_clip");
    expect_near(cfg.action.raw_action_clip, 1.0, "action.raw_action_clip");

    const std::array<double, 3> expected_command_scale = {1.0, 1.0, 1.0};
    const std::array<double, 3> expected_ang_vel_scale = {0.2, 0.2, 0.2};
    expect(cfg.observation_scales.command_scale == expected_command_scale,
           "observation.command_scale");
    expect(cfg.observation_scales.body_ang_vel_scale == expected_ang_vel_scale,
           "observation.body_ang_vel_scale");
    for (std::size_t i = 0; i < 12; ++i) {
        expect_near(cfg.observation_scales.dof_pos_scale[i], 1.0,
                    "observation.dof_pos_scale");
        expect_near(cfg.observation_scales.dof_vel_scale[i], 0.05,
                    "observation.dof_vel_scale");
    }

    for (std::size_t i = 0; i < 4; ++i) {
        expect_near(cfg.ankle_motor_limits.min_rad[i], -1.1,
                    "default ankle_motor_limits.min_rad");
        expect_near(cfg.ankle_motor_limits.max_rad[i], 1.1,
                    "default ankle_motor_limits.max_rad");
    }
    const std::array<double, 2> expected_virtual_kp = {187.0, 187.0};
    const std::array<double, 2> expected_virtual_kd = {9.07, 9.07};
    expect(cfg.ankle_torque.virtual_kp == expected_virtual_kp,
           "ankle_torque.virtual_kp derived from stiffness");
    expect(cfg.ankle_torque.virtual_kd == expected_virtual_kd,
           "ankle_torque.virtual_kd derived from damping");
    expect_near(cfg.ankle_torque.filter_cutoff_rad_s, 100.0,
                "default ankle_torque.filter_cutoff_rad_s");
    expect_near(cfg.ankle_torque.filter_dt_s, 0.001,
                "default ankle_torque.filter_dt_s");
    expect_near(cfg.ankle_torque.motor_rated_torque_nm, 10.5,
                "default ankle_torque.motor_rated_torque_nm");
    expect_near(cfg.ankle_torque.target_torque_limit_permille, 2000.0,
                "default ankle_torque.target_torque_limit_permille");

    expect_near(cfg.sensor_guard.max_imu_sample_age_s, 0.050,
                "default guard.max_imu_sample_age_s");
    expect_near(cfg.sensor_guard.max_motor_sample_age_s, 0.050,
                "default guard.max_motor_sample_age_s");
    expect_near(cfg.sensor_guard.max_sensor_state_skew_s, 0.030,
                "default guard.max_sensor_state_skew_s");

    expect(cfg.recorder.enabled, "default recorder.enabled");
    expect(cfg.recorder.directory ==
               std::filesystem::path(ROBOT_INFERENCE_ROOT_DIR) / "log",
           "default recorder.directory resolved against ROBOT_INFERENCE_ROOT_DIR");
    expect(cfg.recorder.file_prefix.empty(), "default recorder.file_prefix");
    expect(cfg.recorder.flush_interval == std::chrono::milliseconds(1000),
           "default recorder.flush_interval");
    expect(cfg.recorder.max_queue_depth == 4096,
           "default recorder.max_queue_depth");
}

void test_unknown_key_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(read_deploy_yaml(),
                                           "step_dt: 0.02\n",
                                           "step_dt: 0.02\nbogus_key: 42\n");
    expect(!load_yaml_string(yaml, cfg, error), "unknown key must fail");
    expect(error.find("bogus_key") != std::string::npos,
           "unknown-key error should name the key: " + error);
}

void test_wrong_array_length_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(
        read_deploy_yaml(),
        "joint_ids_map: [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]",
        "joint_ids_map: [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5]");
    expect(!load_yaml_string(yaml, cfg, error),
           "11-element joint_ids_map must fail");
}

void test_joint_ids_map_permutation_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(
        read_deploy_yaml(),
        "joint_ids_map: [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11]",
        "joint_ids_map: [0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 10]");
    expect(!load_yaml_string(yaml, cfg, error),
           "duplicate joint_ids_map entry must fail");
    expect(error.find("permutation") != std::string::npos ||
               error.find("duplicate") != std::string::npos,
           "permutation error should be clear: " + error);
}

void test_default_joint_pos_offset_mismatch_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(
        read_deploy_yaml(),
        "default_joint_pos: [0.0, 0.0, -0.2, -0.2, 0.0, 0.0, 0.2, 0.2, -0.05, -0.05, 0.0, 0.0]",
        "default_joint_pos: [0.01, 0.0, -0.2, -0.2, 0.0, 0.0, 0.2, 0.2, -0.05, -0.05, 0.0, 0.0]");
    expect(!load_yaml_string(yaml, cfg, error),
           "root default_joint_pos and action offset mismatch must fail");
    expect(error.find("offset") != std::string::npos,
           "mismatch error should name offset: " + error);
}

void test_non_null_observation_clip_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(read_deploy_yaml(),
                                           "    clip: null\n",
                                           "    clip: [-1.0, 1.0]\n");
    expect(!load_yaml_string(yaml, cfg, error),
           "non-null observation clip must fail");
    expect(error.find("clip") != std::string::npos,
           "clip error should name the key: " + error);
}

void test_ankle_gain_mismatch_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_first(read_deploy_yaml(),
                                           "  187.0, 187.0]\n",
                                           "  187.0, 188.0]\n");
    expect(!load_yaml_string(yaml, cfg, error),
           "inconsistent ankle stiffness must fail");
    expect(error.find("ankle") != std::string::npos,
           "ankle gain error should name ankle motors: " + error);
}

}  // namespace

int main()
{
    test_load_real_deploy_yaml();
    test_unknown_key_fails();
    test_wrong_array_length_fails();
    test_joint_ids_map_permutation_fails();
    test_default_joint_pos_offset_mismatch_fails();
    test_non_null_observation_clip_fails();
    test_ankle_gain_mismatch_fails();

    std::cout << "deploy_config_test passed\n";
    return 0;
}
