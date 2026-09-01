#include "kinematics/ankle_motor_ik.hpp"
#include "robot/action_processor.hpp"
#include "robot/joint_mapping.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr int kDof = 12;
constexpr int kLeftPitchDof = 8;
constexpr int kRightPitchDof = 9;
constexpr int kLeftRollDof = 10;
constexpr int kRightRollDof = 11;

constexpr int kLeftUpperMotor = 4;
constexpr int kLeftLowerMotor = 5;
constexpr int kRightUpperMotor = 10;
constexpr int kRightLowerMotor = 11;

constexpr double kTolerance = 1e-9;

const std::array<int, 4> kAnkleMotorIndices = {
    kLeftUpperMotor,
    kLeftLowerMotor,
    kRightUpperMotor,
    kRightLowerMotor,
};

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
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

double deg_to_rad(double degrees)
{
    return ankle_motor_ik::deg_to_rad(degrees);
}

inference::JointMappingConfig make_mapping_config()
{
    inference::JointMappingConfig config;
    config.model_to_motor_index = {0, 6, 1, 7, 2, 8, 3, 9};
    config.left_ankle_parallel = {8, 10, 4, 5};
    config.right_ankle_parallel = {9, 11, 10, 11};
    config.motor_to_model_direction = {
        -1, -1, 1,  1, -1, -1,
        -1,  1, 1, -1, -1, -1
    };
    return config;
}

std::shared_ptr<const inference::robot_detail::JointMapping> make_mapping()
{
    std::string error;
    auto mapping = inference::robot_detail::JointMapping::create(
        kDof,
        make_mapping_config(),
        error);
    expect(static_cast<bool>(mapping), "joint mapping should be valid: " + error);
    return mapping;
}

inference::PolicyConfig make_policy_config()
{
    inference::PolicyConfig config;
    config.stand_pose_rad.assign(kDof, 0.0);
    return config;
}

inference::AnkleMotorLimitConfig make_ankle_motor_limits()
{
    inference::AnkleMotorLimitConfig limits;
    limits.min_rad = {-6.0, -6.0, -6.0, -6.0};
    limits.max_rad = { 6.0,  6.0,  6.0,  6.0};
    return limits;
}

inference::robot_detail::ActionProcessor make_processor()
{
    return inference::robot_detail::ActionProcessor(
        make_mapping(),
        make_policy_config(),
        make_ankle_motor_limits(),
        std::vector<double>(kDof, 0.0),
        std::vector<double>(kDof, 0.0),
        inference::AnkleTorqueControlConfig{});
}

std::vector<double> make_model_target(double pitch_deg, double roll_deg)
{
    std::vector<double> target(kDof, 0.0);
    target[static_cast<std::size_t>(kLeftPitchDof)] = deg_to_rad(pitch_deg);
    target[static_cast<std::size_t>(kRightPitchDof)] = deg_to_rad(pitch_deg);
    target[static_cast<std::size_t>(kLeftRollDof)] = deg_to_rad(roll_deg);
    target[static_cast<std::size_t>(kRightRollDof)] = deg_to_rad(roll_deg);
    return target;
}

void expect_ankle_targets_near(const std::vector<double>& actual,
                               const std::vector<double>& expected,
                               const std::string& message)
{
    expect(actual.size() == expected.size(), message + ": size mismatch");
    for (int motor_index : kAnkleMotorIndices) {
        expect_near(actual[static_cast<std::size_t>(motor_index)],
                    expected[static_cast<std::size_t>(motor_index)],
                    kTolerance,
                    message + " motor " + std::to_string(motor_index));
    }
}

void test_first_unreachable_fails()
{
    auto processor = make_processor();
    const std::vector<double> target = make_model_target(45.0, 0.0);

    const ankle_motor_ik::MotorAngles raw_result =
        ankle_motor_ik::solve(0.0, deg_to_rad(45.0));
    expect(!raw_result.reachable(), "test pose should be IK unreachable");

    std::vector<double> motor_targets;
    std::string error;
    const bool ok = processor.build_motor_targets(target, motor_targets, error);

    expect(!ok, "first unreachable IK target should fail closed");
    expect(!error.empty(), "first unreachable IK target should report an error");
    expect(error.find("left ankle IK unreachable") != std::string::npos,
           "error should include ankle side");
}

void test_unreachable_after_valid_solution_holds_last()
{
    auto processor = make_processor();
    const std::vector<double> reachable_target = make_model_target(0.0, 30.0);
    const std::vector<double> unreachable_target = make_model_target(45.0, 0.0);

    const ankle_motor_ik::MotorAngles raw_reachable =
        ankle_motor_ik::solve(deg_to_rad(30.0), 0.0);
    expect(raw_reachable.reachable(), "initial hold-last seed pose should be reachable");

    std::vector<double> valid_motor_targets;
    std::string error;
    expect(processor.build_motor_targets(reachable_target,
                                         valid_motor_targets,
                                         error),
           "reachable IK target should succeed: " + error);

    bool nonzero_ankle_target = false;
    for (int motor_index : kAnkleMotorIndices) {
        nonzero_ankle_target =
            nonzero_ankle_target ||
            std::abs(valid_motor_targets[static_cast<std::size_t>(motor_index)]) > 1e-6;
    }
    expect(nonzero_ankle_target, "valid ankle IK target should not be all zeros");

    std::vector<double> held_motor_targets;
    error.clear();
    expect(processor.build_motor_targets(unreachable_target,
                                         held_motor_targets,
                                         error),
           "unreachable IK after a valid solution should hold last: " + error);
    expect_ankle_targets_near(held_motor_targets,
                              valid_motor_targets,
                              "held ankle targets should match last valid solution");
}

void test_partial_unreachable_does_not_pollute_solver_history()
{
    const std::vector<double> partial_unreachable_target =
        make_model_target(65.0, -70.0);
    const std::vector<double> next_reachable_target =
        make_model_target(-20.0, -180.0);

    const ankle_motor_ik::MotorAngles partial_raw =
        ankle_motor_ik::solve(deg_to_rad(-70.0), deg_to_rad(65.0));
    expect(!partial_raw.reachable(),
           "partial test pose should be IK unreachable");
    expect(partial_raw.motor1_reachable != partial_raw.motor2_reachable,
           "partial test pose should have exactly one reachable motor");

    auto processor_after_partial = make_processor();
    std::vector<double> ignored_targets;
    std::string error;
    expect(!processor_after_partial.build_motor_targets(partial_unreachable_target,
                                                        ignored_targets,
                                                        error),
           "partial first IK target should fail closed");

    std::vector<double> after_partial_targets;
    error.clear();
    expect(processor_after_partial.build_motor_targets(next_reachable_target,
                                                       after_partial_targets,
                                                       error),
           "reachable IK after partial failure should succeed: " + error);

    auto fresh_processor = make_processor();
    std::vector<double> fresh_targets;
    error.clear();
    expect(fresh_processor.build_motor_targets(next_reachable_target,
                                               fresh_targets,
                                               error),
           "fresh reachable IK target should succeed: " + error);

    expect_ankle_targets_near(after_partial_targets,
                              fresh_targets,
                              "partial failure should not change later IK branch");
}

}  // namespace

int main()
{
    test_first_unreachable_fails();
    test_unreachable_after_valid_solution_holds_last();
    test_partial_unreachable_does_not_pollute_solver_history();

    std::cout << "action_processor_ankle_ik_test passed\n";
    return 0;
}
