#include "robot/action_processor.hpp"
#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_ik.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"
#include "robot/robot_motor_session.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
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

void expect_near(double actual, double expected, const std::string& message)
{
    if (std::abs(actual - expected) > 1e-9) {
        std::cerr << "FAIL: " << message
                  << " expected=" << expected
                  << " actual=" << actual << "\n";
        std::exit(1);
    }
}

void expect_near(double actual,
                 double expected,
                 double tolerance,
                 const std::string& message)
{
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message
                  << " expected=" << expected
                  << " actual=" << actual
                  << " tolerance=" << tolerance << "\n";
        std::exit(1);
    }
}

inference::JointMappingConfig policy_example_config()
{
    inference::JointMappingConfig config;
    config.model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};
    config.left_ankle_parallel = {8, 10, 4, 5};
    config.right_ankle_parallel = {9, 11, 10, 11};
    config.motor_to_model_direction = {
         1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };
    return config;
}

std::shared_ptr<const inference::robot_detail::JointMapping> make_mapping()
{
    std::string error;
    auto mapping =
        inference::robot_detail::JointMapping::create(12, policy_example_config(), error);
    expect(static_cast<bool>(mapping), "mapping should configure: " + error);
    return mapping;
}

inference::PolicyConfig make_policy_config()
{
    inference::PolicyConfig config;
    config.stand_pose_rad.assign(12, 0.0);
    config.joint_min_rad.assign(12, -1.0);
    config.joint_max_rad.assign(12, 1.0);
    return config;
}

inference::MotorStateSnapshot make_motor_state()
{
    inference::MotorStateSnapshot state;
    state.position_rad.assign(12, 0.0);
    state.velocity_rad_s.assign(12, 0.0);
    state.torque_percent.assign(12, 0.0);
    state.comm_ok.assign(12, 1U);
    state.enabled.assign(12, 1U);
    state.faulted.assign(12, 0U);
    return state;
}

inference::AnkleTorqueControlConfig make_torque_config()
{
    inference::AnkleTorqueControlConfig config;
    config.target_torque_limit_permille = 1000.0;
    return config;
}

std::vector<double> make_motor_kp()
{
    std::vector<double> kp(12, 0.0);
    for (std::size_t i = 0; i < kp.size(); ++i) {
        kp[i] = 10.0 + static_cast<double>(i);
    }
    return kp;
}

std::vector<double> make_motor_kd()
{
    std::vector<double> kd(12, 0.0);
    for (std::size_t i = 0; i < kd.size(); ++i) {
        kd[i] = 0.1 + static_cast<double>(i) * 0.01;
    }
    return kd;
}

void set_ankle_state(inference::MotorStateSnapshot& motor_state,
                     const inference::robot_detail::JointMapping& mapping,
                     const inference::AnkleParallelMap& ankle_map,
                     double pitch,
                     double roll)
{
    const ankle_motor_ik::MotorAngles motors =
        ankle_motor_ik::solve(roll, pitch);
    expect(motors.reachable(), "ankle IK should solve test state");

    const int upper_direction =
        mapping.direction_for_motor(ankle_map.upper_motor_index);
    const int lower_direction =
        mapping.direction_for_motor(ankle_map.lower_motor_index);
    motor_state.position_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)] =
        upper_direction * motors.motor1;
    motor_state.position_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)] =
        lower_direction * motors.motor2;
    motor_state.velocity_rad_s[static_cast<std::size_t>(ankle_map.upper_motor_index)] = 0.0;
    motor_state.velocity_rad_s[static_cast<std::size_t>(ankle_map.lower_motor_index)] = 0.0;
}

struct LowPass2Coefficients {
    double b0 = 0.0;
    double b1 = 0.0;
    double b2 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
};

LowPass2Coefficients compute_low_pass_coefficients(
    const inference::AnkleTorqueControlConfig& config)
{
    const double ts = config.filter_dt_s;
    const double wc = config.filter_cutoff_rad_s;
    const double ts2wc2 = ts * ts * wc * wc;
    const double d = 2500.0 * ts2wc2 + 7071.0 * ts * wc + 10000.0;

    LowPass2Coefficients coeffs;
    coeffs.b0 = 2500.0 * ts2wc2 / d;
    coeffs.b1 = 5000.0 * ts2wc2 / d;
    coeffs.b2 = 2500.0 * ts2wc2 / d;
    coeffs.a1 = -(5000.0 * ts2wc2 - 20000.0) / d;
    coeffs.a2 = -(2500.0 * ts2wc2 - 7071.0 * ts * wc + 10000.0) / d;
    return coeffs;
}

double clamp_symmetric(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

void test_build_motor_targets()
{
    inference::robot_detail::ActionProcessor processor(make_mapping(),
                                                       make_policy_config());
    std::vector<double> model_targets(12, 0.0);
    model_targets[0] = 2.0;    // clipped to +1.0, motor 0 direction +1
    model_targets[1] = 0.25;   // motor 6 direction -1
    model_targets[2] = -0.30;  // motor 1 direction -1
    model_targets[8] = -0.05;  // left ankle pitch, solved through IK
    model_targets[10] = 0.04;  // left ankle roll, solved through IK

    std::vector<double> motor_targets;
    std::string error;
    expect(processor.build_motor_targets(model_targets, motor_targets, error),
           "build_motor_targets should succeed: " + error);
    expect(motor_targets.size() == 12, "motor target size should match DOF count");
    expect_near(motor_targets[0], 1.0, "model 0 should clip and map to motor 0");
    expect_near(motor_targets[6], -0.25, "model 1 should map through motor 6 direction");
    expect_near(motor_targets[1], 0.30, "model 2 should map through motor 1 direction");

    const ankle_motor_ik::MotorAngles left_ankle =
        ankle_motor_ik::solve(model_targets[10], model_targets[8]);
    expect(left_ankle.reachable(), "left ankle IK should solve manual target");
    expect_near(motor_targets[4], -left_ankle.motor1,
                "manual path should command left upper ankle IK motor position");
    expect_near(motor_targets[5], -left_ankle.motor2,
                "manual path should command left lower ankle IK motor position");
}

void test_target_size_validation()
{
    inference::robot_detail::ActionProcessor processor(make_mapping(),
                                                       make_policy_config());
    std::vector<double> motor_targets;
    std::string error;
    expect(!processor.build_motor_targets(std::vector<double>(11, 0.0),
                                          motor_targets,
                                          error),
           "wrong target size should be rejected");
    expect(!error.empty(), "wrong target size should produce an error");
}

void test_reset_start_model_pose()
{
    inference::robot_detail::ActionProcessor processor(make_mapping(),
                                                       make_policy_config());
    std::vector<double> current_motor_q(12, 0.0);
    current_motor_q[0] = 0.40;
    current_motor_q[6] = -0.30;
    std::vector<double> target_model_q(12, 0.0);
    std::vector<double> start_model_q;
    std::string error;

    expect(processor.build_reset_start_model_pose(current_motor_q,
                                                  target_model_q,
                                                  start_model_q,
                                                  error),
           "reset start model pose should build: " + error);
    expect(start_model_q.size() == 12, "reset model pose size should match DOF count");
    expect_near(start_model_q[0], 0.40, "motor 0 should map to model 0");
    expect_near(start_model_q[1], 0.30, "motor 6 direction should map to model 1");
}

void test_policy_impedance_command_uses_ankle_torque()
{
    const auto mapping = make_mapping();
    inference::robot_detail::ActionProcessor processor(mapping,
                                                       make_policy_config());
    inference::MotorStateSnapshot motor_state = make_motor_state();

    constexpr double pitch = -0.05;
    constexpr double roll = 0.04;
    const ankle_motor_ik::MotorAngles motors =
        ankle_motor_ik::solve(roll, pitch);
    expect(motors.reachable(), "left ankle IK should solve policy torque test pose");

    constexpr int upper_motor_index = 4;
    constexpr int lower_motor_index = 5;
    constexpr int pitch_model_index = 8;
    constexpr int roll_model_index = 10;
    const int upper_direction = mapping->direction_for_motor(upper_motor_index);
    const int lower_direction = mapping->direction_for_motor(lower_motor_index);

    motor_state.position_rad[upper_motor_index] = upper_direction * motors.motor1;
    motor_state.position_rad[lower_motor_index] = lower_direction * motors.motor2;
    motor_state.velocity_rad_s[upper_motor_index] = 0.0;
    motor_state.velocity_rad_s[lower_motor_index] = 0.0;

    std::vector<double> model_targets(12, 0.0);
    model_targets[0] = 0.25;
    model_targets[1] = 0.30;
    model_targets[pitch_model_index] = pitch + 0.10;
    model_targets[roll_model_index] = roll - 0.05;

    inference::robot_detail::ActionProcessor::PolicyMotorCommand command;
    std::string error;
    const inference::AnkleTorqueControlConfig torque_config = make_torque_config();
    const std::vector<double> kp = make_motor_kp();
    const std::vector<double> kd = make_motor_kd();
    expect(processor.build_policy_impedance_command(model_targets,
                                                    motor_state,
                                                    kp,
                                                    kd,
                                                    torque_config,
                                                    command,
                                                    error),
           "policy impedance command should build: " + error);
    expect(command.setpoints.size() == 12, "policy setpoint size should match DOF count");
    expect(command.target_effort_permille.size() == 12,
           "policy target effort size should match DOF count");

    expect_near(command.setpoints[0].position_rad,
                model_targets[0],
                "direct motor should use mapped policy target position");
    expect_near(command.setpoints[0].kp, kp[0], "direct motor should keep MIT kp");
    expect_near(command.setpoints[0].kd, kd[0], "direct motor should keep MIT kd");
    expect_near(command.setpoints[0].effort_ff, 0.0,
                "direct motor should not add feed-forward effort");
    expect_near(command.setpoints[6].position_rad,
                -model_targets[1],
                "direct motor should apply motor direction");

    ankle_motor_fk::Solver fk_solver;
    const ankle_motor_fk::FootAngles foot =
        fk_solver.solve(motors.motor1, motors.motor2);
    expect(foot.reachable, "expected ankle FK should solve");

    ankle_motor_jacobian::Result jacobian;
    std::string jacobian_error;
    expect(ankle_motor_jacobian::solve(foot.pitch,
                                       foot.roll,
                                       motors.motor1,
                                       motors.motor2,
                                       jacobian,
                                       jacobian_error),
           "expected ankle jacobian should solve: " + jacobian_error);

    const double pitch_torque_des =
        torque_config.virtual_kp[0] *
        (model_targets[pitch_model_index] - foot.pitch);
    const double roll_torque_des =
        torque_config.virtual_kp[1] *
        (model_targets[roll_model_index] - foot.roll);
    const LowPass2Coefficients coeffs =
        compute_low_pass_coefficients(torque_config);
    const double pitch_torque_lp = coeffs.b0 * pitch_torque_des;
    const double roll_torque_lp = coeffs.b0 * roll_torque_des;

    const double upper_torque_nm =
        jacobian.virtual_from_motor[0][0] * pitch_torque_lp +
        jacobian.virtual_from_motor[1][0] * roll_torque_lp;
    const double lower_torque_nm =
        jacobian.virtual_from_motor[0][1] * pitch_torque_lp +
        jacobian.virtual_from_motor[1][1] * roll_torque_lp;
    const double scale = 1000.0 / torque_config.motor_rated_torque_nm;
    const double expected_upper_effort = clamp_symmetric(
        upper_direction * upper_torque_nm * scale,
        torque_config.target_torque_limit_permille);
    const double expected_lower_effort = clamp_symmetric(
        lower_direction * lower_torque_nm * scale,
        torque_config.target_torque_limit_permille);

    expect_near(command.setpoints[upper_motor_index].position_rad,
                motor_state.position_rad[upper_motor_index],
                "ankle torque path should use current upper motor position placeholder");
    expect_near(command.setpoints[lower_motor_index].position_rad,
                motor_state.position_rad[lower_motor_index],
                "ankle torque path should use current lower motor position placeholder");
    expect_near(command.setpoints[upper_motor_index].kp, 0.0,
                "ankle upper motor kp should be zero in torque path");
    expect_near(command.setpoints[upper_motor_index].kd, 0.0,
                "ankle upper motor kd should be zero in torque path");
    expect_near(command.setpoints[lower_motor_index].kp, 0.0,
                "ankle lower motor kp should be zero in torque path");
    expect_near(command.setpoints[lower_motor_index].kd, 0.0,
                "ankle lower motor kd should be zero in torque path");
    expect_near(command.setpoints[upper_motor_index].effort_ff,
                expected_upper_effort,
                1e-9,
                "upper ankle effort should match filtered J^T virtual torque");
    expect_near(command.setpoints[lower_motor_index].effort_ff,
                expected_lower_effort,
                1e-9,
                "lower ankle effort should match filtered J^T virtual torque");
    expect_near(command.target_effort_permille[upper_motor_index],
                expected_upper_effort,
                "upper ankle logged effort should match command effort");
    expect_near(command.target_effort_permille[lower_motor_index],
                expected_lower_effort,
                "lower ankle logged effort should match command effort");
}

void test_policy_ankle_torque_clamps_to_config_limit()
{
    const auto mapping = make_mapping();
    inference::robot_detail::ActionProcessor processor(mapping,
                                                       make_policy_config());
    inference::MotorStateSnapshot motor_state = make_motor_state();

    constexpr double pitch = -0.05;
    constexpr double roll = 0.04;
    const ankle_motor_ik::MotorAngles motors =
        ankle_motor_ik::solve(roll, pitch);
    expect(motors.reachable(), "left ankle IK should solve clamp test pose");

    constexpr int upper_motor_index = 4;
    constexpr int lower_motor_index = 5;
    constexpr int pitch_model_index = 8;
    constexpr int roll_model_index = 10;
    const int upper_direction = mapping->direction_for_motor(upper_motor_index);
    const int lower_direction = mapping->direction_for_motor(lower_motor_index);
    motor_state.position_rad[upper_motor_index] = upper_direction * motors.motor1;
    motor_state.position_rad[lower_motor_index] = lower_direction * motors.motor2;

    std::vector<double> model_targets(12, 0.0);
    model_targets[pitch_model_index] = 0.95;
    model_targets[roll_model_index] = -0.95;

    inference::AnkleTorqueControlConfig torque_config = make_torque_config();
    torque_config.virtual_kp = {10000.0, 10000.0};
    torque_config.target_torque_limit_permille = 1.0;

    inference::robot_detail::ActionProcessor::PolicyMotorCommand command;
    std::string error;
    expect(processor.build_policy_impedance_command(model_targets,
                                                    motor_state,
                                                    make_motor_kp(),
                                                    make_motor_kd(),
                                                    torque_config,
                                                    command,
                                                    error),
           "policy impedance command should build for clamp test: " + error);

    const double upper_effort =
        command.setpoints[upper_motor_index].effort_ff;
    const double lower_effort =
        command.setpoints[lower_motor_index].effort_ff;
    expect(std::abs(upper_effort) <= torque_config.target_torque_limit_permille,
           "upper ankle effort should respect configured clamp");
    expect(std::abs(lower_effort) <= torque_config.target_torque_limit_permille,
           "lower ankle effort should respect configured clamp");
    expect(std::abs(upper_effort) == torque_config.target_torque_limit_permille ||
           std::abs(lower_effort) == torque_config.target_torque_limit_permille,
           "at least one ankle effort should hit the configured clamp");
}

void test_policy_ankle_pitch_hard_limit_rejects_current_state()
{
    const auto mapping = make_mapping();
    inference::PolicyConfig policy_config = make_policy_config();
    policy_config.joint_max_rad[8] = 0.02;
    inference::robot_detail::ActionProcessor processor(mapping, policy_config);
    inference::MotorStateSnapshot motor_state = make_motor_state();
    set_ankle_state(motor_state,
                    *mapping,
                    mapping->left_ankle(),
                    0.05,
                    0.0);

    inference::robot_detail::ActionProcessor::PolicyMotorCommand command;
    std::string error;
    expect(!processor.build_policy_impedance_command(std::vector<double>(12, 0.0),
                                                     motor_state,
                                                     make_motor_kp(),
                                                     make_motor_kd(),
                                                     make_torque_config(),
                                                     command,
                                                     error),
           "current pitch beyond hard limit should reject policy command");
    expect(error.find("left ankle") != std::string::npos,
           "pitch hard-limit error should include ankle side");
    expect(error.find("pitch") != std::string::npos,
           "pitch hard-limit error should include axis");
    expect(error.find("dof 8") != std::string::npos,
           "pitch hard-limit error should include model DOF");
}

void test_policy_ankle_roll_hard_limit_rejects_current_state()
{
    const auto mapping = make_mapping();
    inference::PolicyConfig policy_config = make_policy_config();
    policy_config.joint_min_rad[10] = -0.02;
    inference::robot_detail::ActionProcessor processor(mapping, policy_config);
    inference::MotorStateSnapshot motor_state = make_motor_state();
    set_ankle_state(motor_state,
                    *mapping,
                    mapping->left_ankle(),
                    0.0,
                    -0.05);

    inference::robot_detail::ActionProcessor::PolicyMotorCommand command;
    std::string error;
    expect(!processor.build_policy_impedance_command(std::vector<double>(12, 0.0),
                                                     motor_state,
                                                     make_motor_kp(),
                                                     make_motor_kd(),
                                                     make_torque_config(),
                                                     command,
                                                     error),
           "current roll beyond hard limit should reject policy command");
    expect(error.find("left ankle") != std::string::npos,
           "roll hard-limit error should include ankle side");
    expect(error.find("roll") != std::string::npos,
           "roll hard-limit error should include axis");
    expect(error.find("dof 10") != std::string::npos,
           "roll hard-limit error should include model DOF");
}

void test_policy_target_limit_does_not_trip_current_state_hard_limit()
{
    const auto mapping = make_mapping();
    inference::PolicyConfig policy_config = make_policy_config();
    policy_config.joint_min_rad[8] = -0.05;
    policy_config.joint_max_rad[8] = 0.05;
    policy_config.joint_min_rad[10] = -0.05;
    policy_config.joint_max_rad[10] = 0.05;
    inference::robot_detail::ActionProcessor processor(mapping, policy_config);
    inference::MotorStateSnapshot motor_state = make_motor_state();
    set_ankle_state(motor_state,
                    *mapping,
                    mapping->left_ankle(),
                    0.0,
                    0.0);

    std::vector<double> model_targets(12, 0.0);
    model_targets[8] = 1.0;
    model_targets[10] = -1.0;

    inference::robot_detail::ActionProcessor::PolicyMotorCommand command;
    std::string error;
    expect(processor.build_policy_impedance_command(model_targets,
                                                    motor_state,
                                                    make_motor_kp(),
                                                    make_motor_kd(),
                                                    make_torque_config(),
                                                    command,
                                                    error),
           "target beyond relative limits should clip without tripping current-state hard limit: " +
           error);
    expect(command.setpoints.size() == 12,
           "clipped target policy command should still produce all motor setpoints");
}

}  // namespace

int main()
{
    test_build_motor_targets();
    test_target_size_validation();
    test_reset_start_model_pose();
    test_policy_impedance_command_uses_ankle_torque();
    test_policy_ankle_torque_clamps_to_config_limit();
    test_policy_ankle_pitch_hard_limit_rejects_current_state();
    test_policy_ankle_roll_hard_limit_rejects_current_state();
    test_policy_target_limit_does_not_trip_current_state_hard_limit();

    std::cout << "action_processor_test passed\n";
    return 0;
}
