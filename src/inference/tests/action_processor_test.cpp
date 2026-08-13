#include "robot/action_processor.hpp"

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

void test_build_motor_targets()
{
    inference::robot_detail::ActionProcessor processor(make_mapping(),
                                                       make_policy_config());
    std::vector<double> model_targets(12, 0.0);
    model_targets[0] = 2.0;    // clipped to +1.0, motor 0 direction +1
    model_targets[1] = 0.25;   // motor 6 direction -1
    model_targets[2] = -0.30;  // motor 1 direction -1

    std::vector<double> motor_targets;
    std::string error;
    expect(processor.build_motor_targets(model_targets, motor_targets, error),
           "build_motor_targets should succeed: " + error);
    expect(motor_targets.size() == 12, "motor target size should match DOF count");
    expect_near(motor_targets[0], 1.0, "model 0 should clip and map to motor 0");
    expect_near(motor_targets[6], -0.25, "model 1 should map through motor 6 direction");
    expect_near(motor_targets[1], 0.30, "model 2 should map through motor 1 direction");
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
    std::vector<double> q0_motor(12, 0.0);
    q0_motor[0] = 0.40;
    q0_motor[6] = -0.30;
    std::vector<double> target_model(12, 0.0);
    std::vector<double> q0_model;
    std::string error;

    expect(processor.build_reset_start_model_pose(q0_motor,
                                                  target_model,
                                                  q0_model,
                                                  error),
           "reset start model pose should build: " + error);
    expect(q0_model.size() == 12, "reset model pose size should match DOF count");
    expect_near(q0_model[0], 0.40, "motor 0 should map to model 0");
    expect_near(q0_model[1], 0.30, "motor 6 direction should map to model 1");
}

}  // namespace

int main()
{
    test_build_motor_targets();
    test_target_size_validation();
    test_reset_start_model_pose();

    std::cout << "action_processor_test passed\n";
    return 0;
}
