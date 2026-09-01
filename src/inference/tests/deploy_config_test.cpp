#include "config/deploy_config.hpp"
#include "policy/policy_observation_config.hpp"

#include <array>
#include <chrono>
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

std::string replace_all(std::string text,
                        const std::string& from,
                        const std::string& to)
{
    expect(text.find(from) != std::string::npos,
           "replacement source not found in deploy.yaml: " + from);
    std::size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
    return text;
}

/* 正向回归：加载真实 deploy.yaml，断言全部字段等于重构前
   policy_test.cpp 的硬编码值（行为等价）。
   675 编译分支下先移除仅 705 使用的 gait_phase 段。 */
void test_load_real_deploy_yaml()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    bool loaded;
    if constexpr (inference::policy_observation::kEnableGaitPhase) {
        loaded = inference::load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, cfg, error);
    } else {
        const std::string yaml = replace_all(
            read_deploy_yaml(),
            "  gait_phase: { period: 0.74, stand_threshold: 0.05, move_threshold: 0.15 }\n",
            "");
        loaded = load_yaml_string(yaml, cfg, error);
    }
    expect(loaded, "load_deploy_config failed: " + error);

    // hardware
    expect(cfg.motor.ethercat_ifname == "enp8s0", "ethercat_ifname");
    expect(cfg.imu.type == imu_base::ReaderType::XSENS_MTI_CAN, "imu.type");
    expect(cfg.imu.device == "can0", "imu.device");
    expect(cfg.imu.baudrate == 921600, "imu.baudrate");
    expect(!cfg.imu.print_imu && !cfg.imu.print_ahrs, "imu print flags");

    // 拓扑：模型序→电机序映射与脚踝并联声明
    const std::vector<int> expected_map = {0, 6, 1, 7, 2, 8, 3, 9};
    expect(cfg.joint_mapping.model_to_motor_index == expected_map,
           "joint_mapping.model_to_motor_index");
    const std::array<int, 12> expected_direction = {
        -1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };
    expect(cfg.joint_mapping.motor_to_model_direction == expected_direction,
           "joint_mapping.motor_to_model_direction");
    expect(cfg.joint_mapping.left_ankle_parallel.model_pitch_dof == 8 &&
           cfg.joint_mapping.left_ankle_parallel.model_roll_dof == 10 &&
           cfg.joint_mapping.left_ankle_parallel.upper_motor_index == 4 &&
           cfg.joint_mapping.left_ankle_parallel.lower_motor_index == 5,
           "left_ankle_parallel");
    expect(cfg.joint_mapping.right_ankle_parallel.model_pitch_dof == 9 &&
           cfg.joint_mapping.right_ankle_parallel.model_roll_dof == 11 &&
           cfg.joint_mapping.right_ankle_parallel.upper_motor_index == 10 &&
           cfg.joint_mapping.right_ankle_parallel.lower_motor_index == 11,
           "right_ankle_parallel");

    // motor：物理电机顺序的增益。直驱 250/10；脚踝槽位
    // M4/M5/M10/M11 取 virtual_kp[0]/virtual_kd[0] = 187/9.07
    expect(cfg.motor.num_motors == static_cast<int>(motor_base::kMaxMotors),
           "motor.num_motors");
    expect(cfg.motor.control_mode == motor_base::MotorControlMode::IMPEDANCE,
           "motor.control_mode");
    const std::array<double, 12> expected_kp = {
        250.0, 250.0, 250.0, 250.0, 187.0, 187.0,
        250.0, 250.0, 250.0, 250.0, 187.0, 187.0
    };
    const std::array<double, 12> expected_kd = {
        10.0, 10.0, 10.0, 10.0, 9.07, 9.07,
        10.0, 10.0, 10.0, 10.0, 9.07, 9.07
    };
    expect(cfg.motor.mit_kp == expected_kp, "motor.mit_kp motor-ordered");
    expect(cfg.motor.mit_kd == expected_kd, "motor.mit_kd motor-ordered");
    expect(cfg.motor.wait_all_motors_timeout_ms == 20000, "wait_all_motors_timeout_ms");
    expect(cfg.motor.wait_all_motors_poll_ms == 100, "wait_all_motors_poll_ms");
    expect(cfg.motor.control_ready_timeout_ms == 4000, "control_ready_timeout_ms");
    expect(cfg.motor.print_motors_info, "motor.print_motors_info");
    expect(cfg.motor.print_motor_ids.size() == 12, "motor.print_motor_ids size");
    for (std::size_t i = 0; i < cfg.motor.print_motor_ids.size(); ++i) {
        expect(cfg.motor.print_motor_ids[i] == static_cast<int>(i),
               "motor.print_motor_ids value");
    }

    // policy
    expect(cfg.policy.model_path ==
               std::string(ROBOT_INFERENCE_ROOT_DIR) + "/model/policy.pt",
           "policy.model_path resolved against ROBOT_INFERENCE_ROOT_DIR");
    expect_near(cfg.policy.step_dt, 0.02, "policy.step_dt");
    expect_near(cfg.policy.target_interpolation_duration_s, 0.0,
                "policy.target_interpolation_duration_s");
    if constexpr (inference::policy_observation::kEnableGaitPhase) {
        expect_near(cfg.policy.gait.period, 0.74, "gait.period");
        expect_near(cfg.policy.gait.stand_threshold, 0.05, "gait.stand_threshold");
        expect_near(cfg.policy.gait.move_threshold, 0.15, "gait.move_threshold");
    }

    // action（模型 DOF 顺序）
    const std::array<double, 12> expected_default_pos = {
        0.0, 0.0, -0.2, -0.2, 0.0, 0.0, 0.2, 0.2, -0.05, -0.05, 0.0, 0.0
    };
    const std::array<double, 12> expected_scale = {
        0.16, 0.16, 0.32, 0.32, 0.1, 0.1, 0.36, 0.36, 0.18, 0.18, 0.1, 0.1
    };
    const std::array<std::array<double, 2>, 12> expected_clip = {{
        {-0.22, 0.22}, {-0.22, 0.22}, {-0.28, 0.35}, {-0.28, 0.35},
        {-0.16, 0.16}, {-0.16, 0.16}, {-0.22, 0.38}, {-0.22, 0.38},
        {-0.14, 0.2},  {-0.14, 0.2},  {-0.12, 0.12}, {-0.12, 0.12}
    }};
    expect(cfg.action.default_joint_pos_rad == expected_default_pos,
           "action.default_joint_pos_rad");
    expect(cfg.action.action_scale == expected_scale, "action.action_scale");
    expect(cfg.action.action_clip == expected_clip, "action.action_clip");
    expect_near(cfg.action.raw_action_clip, 1.0, "action.raw_action_clip");

    // observation scales
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

    // ankle
    for (std::size_t i = 0; i < 4; ++i) {
        expect_near(cfg.ankle_motor_limits.min_rad[i], -1.1,
                    "ankle_motor_limits.min_rad");
        expect_near(cfg.ankle_motor_limits.max_rad[i], 1.1,
                    "ankle_motor_limits.max_rad");
    }
    const std::array<double, 2> expected_virtual_kp = {187.0, 187.0};
    const std::array<double, 2> expected_virtual_kd = {9.07, 9.07};
    expect(cfg.ankle_torque.virtual_kp == expected_virtual_kp,
           "ankle_torque.virtual_kp");
    expect(cfg.ankle_torque.virtual_kd == expected_virtual_kd,
           "ankle_torque.virtual_kd");
    expect_near(cfg.ankle_torque.filter_cutoff_rad_s, 100.0,
                "ankle_torque.filter_cutoff_rad_s");
    expect_near(cfg.ankle_torque.filter_dt_s, 0.001, "ankle_torque.filter_dt_s");
    expect_near(cfg.ankle_torque.motor_rated_torque_nm, 10.5,
                "ankle_torque.motor_rated_torque_nm");
    expect_near(cfg.ankle_torque.target_torque_limit_permille, 2000.0,
                "ankle_torque.target_torque_limit_permille");

    // guard
    expect_near(cfg.sensor_guard.max_imu_sample_age_s, 0.050,
                "guard.max_imu_sample_age_s");
    expect_near(cfg.sensor_guard.max_motor_sample_age_s, 0.050,
                "guard.max_motor_sample_age_s");
    expect_near(cfg.sensor_guard.max_sensor_state_skew_s, 0.030,
                "guard.max_sensor_state_skew_s");

    // recorder
    expect(cfg.recorder.enabled, "recorder.enabled");
    expect(cfg.recorder.directory ==
               std::filesystem::path(ROBOT_INFERENCE_ROOT_DIR) / "log",
           "recorder.directory resolved against ROBOT_INFERENCE_ROOT_DIR");
    expect(cfg.recorder.file_prefix.empty(), "recorder.file_prefix");
    expect(cfg.recorder.flush_interval == std::chrono::milliseconds(1000),
           "recorder.flush_interval");
    expect(cfg.recorder.max_queue_depth == 4096, "recorder.max_queue_depth");
}

/* 负向用例的基础文本：真实 deploy.yaml；675 编译分支下先移除
   仅 705 使用的 gait_phase 段，避免干扰其他用例的断言。 */
std::string negative_case_base_yaml()
{
    std::string yaml = read_deploy_yaml();
    if constexpr (!inference::policy_observation::kEnableGaitPhase) {
        yaml = replace_all(
            yaml,
            "  gait_phase: { period: 0.74, stand_threshold: 0.05, move_threshold: 0.15 }\n",
            "");
    }
    return yaml;
}

/* 负向：缺键 */
void test_missing_key_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_all(negative_case_base_yaml(),
                                         "  step_dt: 0.02\n", "");
    expect(!load_yaml_string(yaml, cfg, error),
           "missing policy.step_dt must fail");
    expect(error.find("step_dt") != std::string::npos,
           "missing-key error should name the key: " + error);
}

/* 负向：未知键 */
void test_unknown_key_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_all(negative_case_base_yaml(),
                                         "recorder:\n",
                                         "recorder:\n  bogus_key: 42\n");
    expect(!load_yaml_string(yaml, cfg, error),
           "unknown key must fail");
    expect(error.find("bogus_key") != std::string::npos,
           "unknown-key error should name the key: " + error);
}

/* 负向：数组长度错误（11 项） */
void test_wrong_array_length_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_all(
        negative_case_base_yaml(),
        "motor_to_model_direction: [-1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1]",
        "motor_to_model_direction: [-1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1]");
    expect(!load_yaml_string(yaml, cfg, error),
           "11-element motor_to_model_direction must fail");
}

/* 负向：action clip 下界大于上界 */
void test_clip_lower_above_upper_fails()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;
    const std::string yaml = replace_all(negative_case_base_yaml(),
                                         "[-0.12, 0.12]",
                                         "[0.12, -0.12]");
    expect(!load_yaml_string(yaml, cfg, error),
           "clip lower > upper must fail");
}

/* gait_phase 段与编译开关的一致性（双向） */
void test_gait_phase_branch_consistency()
{
    inference::RobotInterfaceConfig cfg;
    std::string error;

    if constexpr (inference::policy_observation::kEnableGaitPhase) {
        // 705 分支：删除 gait_phase 段必须失败
        const std::string yaml = replace_all(
            read_deploy_yaml(),
            "  gait_phase: { period: 0.74, stand_threshold: 0.05, move_threshold: 0.15 }\n",
            "");
        expect(!load_yaml_string(yaml, cfg, error),
               "705 build must require the gait_phase section");
    } else {
        // 675 分支：携带 gait_phase 段的配置必须失败（真实 deploy.yaml 即如此）
        expect(!load_yaml_string(read_deploy_yaml(), cfg, error),
               "675 build must reject the gait_phase section");
        expect(error.find("gait_phase") != std::string::npos,
               "gait_phase error should name the section: " + error);
    }
}

}  // namespace

int main()
{
    test_load_real_deploy_yaml();
    test_missing_key_fails();
    test_unknown_key_fails();
    test_wrong_array_length_fails();
    test_clip_lower_above_upper_fails();
    test_gait_phase_branch_consistency();

    std::cout << "deploy_config_test passed\n";
    return 0;
}
