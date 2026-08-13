#include "robot/joint_mapping.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
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

void expect_invalid(inference::JointMappingConfig config,
                    const std::string& message)
{
    std::string error;
    const auto mapping = inference::robot_detail::JointMapping::create(12, config, error);
    expect(!mapping, message);
    expect(!error.empty(), message + " should explain the failure");
}

void test_policy_example_mapping()
{
    std::string error;
    const auto mapping =
        inference::robot_detail::JointMapping::create(12, policy_example_config(), error);
    expect(static_cast<bool>(mapping),
           "policy example mapping should configure: " + error);
    expect(mapping->dof_count() == 12, "configured DOF count should be 12");
    const auto& left_ankle = mapping->left_ankle();
    expect(left_ankle.model_pitch_dof == 8, "left ankle pitch DOF should be preserved");
    expect(left_ankle.model_roll_dof == 10, "left ankle roll DOF should be preserved");
    expect(left_ankle.upper_motor_index == 4, "left ankle upper motor should be preserved");
    expect(left_ankle.lower_motor_index == 5, "left ankle lower motor should be preserved");

    const auto& right_ankle = mapping->right_ankle();
    expect(right_ankle.model_pitch_dof == 9, "right ankle pitch DOF should be preserved");
    expect(right_ankle.model_roll_dof == 11, "right ankle roll DOF should be preserved");
    expect(right_ankle.upper_motor_index == 10, "right ankle upper motor should be preserved");
    expect(right_ankle.lower_motor_index == 11, "right ankle lower motor should be preserved");

    const int expected_direct_motors[] = {0, 6, 1, 7, 2, 8, 3, 9};
    for (int model_index = 0; model_index < 8; ++model_index) {
        expect(!mapping->is_parallel_model_dof(model_index),
               "model DOF " + std::to_string(model_index) + " should be direct");
        expect(mapping->direct_motor_for_model_dof(model_index) ==
                   expected_direct_motors[model_index],
               "direct model DOF should map to expected motor");
    }

    for (int model_index : {8, 9, 10, 11}) {
        expect(mapping->is_parallel_model_dof(model_index),
               "ankle model DOF should be marked parallel");
        expect(mapping->direct_motor_for_model_dof(model_index) == -1,
               "parallel model DOF should not have direct motor");
    }
}

void test_direction_mapping()
{
    std::string error;
    auto mapping =
        inference::robot_detail::JointMapping::create(12, policy_example_config(), error);
    expect(static_cast<bool>(mapping),
           "direction mapping should configure: " + error);
    expect(mapping->direction_for_motor(0) == 1, "motor 0 direction should be 1");
    expect(mapping->direction_for_motor(1) == -1, "motor 1 direction should be -1");
    expect(mapping->direction_for_motor(11) == -1, "motor 11 direction should be -1");

    auto default_direction_config = policy_example_config();
    default_direction_config.motor_to_model_direction.clear();
    mapping = inference::robot_detail::JointMapping::create(
        12, default_direction_config, error);
    expect(static_cast<bool>(mapping),
           "empty direction config should configure: " + error);
    for (int motor_index = 0; motor_index < 12; ++motor_index) {
        expect(mapping->direction_for_motor(motor_index) == 1,
               "empty direction config should default each motor to 1");
    }
}

void test_invalid_configs()
{
    auto config = policy_example_config();
    config.left_ankle_parallel.upper_motor_index = 12;
    expect_invalid(config, "out-of-range ankle motor should be rejected");

    config = policy_example_config();
    config.model_to_motor_index[0] = 4;
    expect_invalid(config, "duplicate direct and ankle motor should be rejected");

    config = policy_example_config();
    config.right_ankle_parallel.model_pitch_dof = 8;
    expect_invalid(config, "duplicate ankle model DOF should be rejected");

    config = policy_example_config();
    config.model_to_motor_index.pop_back();
    expect_invalid(config, "wrong direct mapping count should be rejected");

    config = policy_example_config();
    config.motor_to_model_direction[2] = 0;
    expect_invalid(config, "invalid direction value should be rejected");
}

}  // namespace

int main()
{
    test_policy_example_mapping();
    test_direction_mapping();
    test_invalid_configs();

    std::cout << "joint_mapping_test passed\n";
    return 0;
}
