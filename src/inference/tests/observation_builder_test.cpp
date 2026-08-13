#include "robot/observation_builder.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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

void expect_near(float actual, float expected, const std::string& message)
{
    if (std::abs(actual - expected) > 1e-6F) {
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

inference::MotorStateSnapshot make_motor_state()
{
    inference::MotorStateSnapshot state;
    state.position_rad.assign(12, 0.0);
    state.velocity_rad_s.assign(12, 0.0);
    return state;
}

inference::ImuStateSnapshot make_imu_state()
{
    inference::ImuStateSnapshot state;
    state.ahrs_ready = true;
    state.projected_gravity_valid = true;
    state.body_ang_vel = {10.0, -20.0, 30.0};
    state.projected_gravity = {0.1, 0.2, -0.97};
    return state;
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
    config.dof_pos_scale.assign(12, 1.0);
    config.dof_vel_scale.assign(12, 0.1);
    config.command_scale = {2.0, 3.0, 4.0};
    config.body_ang_vel_scale = {0.1, 0.2, 0.3};
    return config;
}

void test_direct_joint_terms()
{
    inference::robot_detail::ObservationBuilder builder(make_mapping(),
                                                        make_policy_config());
    inference::robot_detail::ObservationBuilder::MotorStateArray q{};
    inference::robot_detail::ObservationBuilder::MotorStateArray dq{};
    q.fill(0.0);
    dq.fill(0.0);
    q[0] = 0.50;
    dq[0] = 1.0;
    q[6] = -0.40;
    dq[6] = -2.0;

    inference::robot_detail::ObservationBuilder::JointTermArray joint_pos{};
    inference::robot_detail::ObservationBuilder::JointTermArray joint_vel{};
    std::string error;
    expect(builder.build_joint_terms(q,
                                     dq,
                                     std::chrono::steady_clock::now(),
                                     joint_pos,
                                     joint_vel,
                                     error),
           "build_joint_terms should succeed: " + error);

    expect_near(joint_pos[0], 0.50F, "motor 0 should map to model 0 position");
    expect_near(joint_vel[0], 0.10F, "motor 0 velocity should scale");
    expect_near(joint_pos[1], 0.40F, "motor 6 direction should map to model 1 position");
    expect_near(joint_vel[1], 0.20F, "motor 6 direction should map to model 1 velocity");

    builder.reset_runtime_state();
    expect(builder.build_joint_terms(q,
                                     dq,
                                     std::chrono::steady_clock::now(),
                                     joint_pos,
                                     joint_vel,
                                     error),
           "build_joint_terms should succeed after reset: " + error);
}

void test_build_full_terms()
{
    inference::robot_detail::ObservationBuilder builder(make_mapping(),
                                                        make_policy_config());
    inference::MotorStateSnapshot motor_state = make_motor_state();
    motor_state.position_rad[0] = 0.50;
    motor_state.velocity_rad_s[0] = 1.0;
    motor_state.position_rad[6] = -0.40;
    motor_state.velocity_rad_s[6] = -2.0;

    inference::PolicyAction last_action{};
    last_action.fill(0.0F);
    last_action[3] = 0.25F;

    inference::PolicyObservationTerms terms;
    std::string error;
    expect(builder.build(motor_state,
                         make_imu_state(),
                         {1.0, -2.0, 0.5},
                         last_action,
                         terms,
                         error),
           "build should succeed: " + error);

    expect_near(terms.velocity_commands[0], 2.0F, "vx should scale");
    expect_near(terms.velocity_commands[1], -6.0F, "vy should scale");
    expect_near(terms.velocity_commands[2], 2.0F, "yaw rate should scale");
    expect_near(terms.base_ang_vel[0], 1.0F, "roll rate should scale");
    expect_near(terms.base_ang_vel[1], -4.0F, "pitch rate should scale");
    expect_near(terms.base_ang_vel[2], 9.0F, "heading rate should scale");
    expect_near(terms.projected_gravity[0], 0.1F, "projected gravity x should pass through");
    expect_near(terms.projected_gravity[1], 0.2F, "projected gravity y should pass through");
    expect_near(terms.projected_gravity[2], -0.97F, "projected gravity z should pass through");
    expect_near(terms.joint_pos_rel[0], 0.50F, "motor 0 should map to model 0 position");
    expect_near(terms.joint_vel_rel[0], 0.10F, "motor 0 velocity should scale");
    expect_near(terms.joint_pos_rel[1], 0.40F, "motor 6 direction should map to model 1 position");
    expect_near(terms.joint_vel_rel[1], 0.20F, "motor 6 direction should map to model 1 velocity");
    expect_near(terms.last_action[3], 0.25F, "last action should be copied into terms");
}

void test_build_rejects_invalid_inputs()
{
    inference::robot_detail::ObservationBuilder builder(make_mapping(),
                                                        make_policy_config());
    inference::PolicyAction last_action{};
    inference::PolicyObservationTerms terms;
    std::string error;

    inference::ImuStateSnapshot imu_state = make_imu_state();
    imu_state.ahrs_ready = false;
    expect(!builder.build(make_motor_state(),
                          imu_state,
                          {0.0, 0.0, 0.0},
                          last_action,
                          terms,
                          error),
           "build should reject IMU without AHRS data");

    imu_state = make_imu_state();
    imu_state.projected_gravity_valid = false;
    expect(!builder.build(make_motor_state(),
                          imu_state,
                          {0.0, 0.0, 0.0},
                          last_action,
                          terms,
                          error),
           "build should reject invalid projected gravity");

    expect(!builder.build(make_motor_state(),
                          make_imu_state(),
                          {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0},
                          last_action,
                          terms,
                          error),
           "build should reject non-finite velocity commands");

    inference::MotorStateSnapshot motor_state = make_motor_state();
    motor_state.position_rad.pop_back();
    expect(!builder.build(motor_state,
                          make_imu_state(),
                          {0.0, 0.0, 0.0},
                          last_action,
                          terms,
                          error),
           "build should reject motor state size mismatch");
}

}  // namespace

int main()
{
    test_direct_joint_terms();
    test_build_full_terms();
    test_build_rejects_invalid_inputs();

    std::cout << "observation_builder_test passed\n";
    return 0;
}
