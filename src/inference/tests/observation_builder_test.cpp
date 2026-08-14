#include "robot/observation_builder.hpp"

#include "kinematics/ankle_motor_ik.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"

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
    inference::MotorStateSnapshot motor_state = make_motor_state();
    motor_state.position_rad[0] = 0.50;
    motor_state.velocity_rad_s[0] = 1.0;
    motor_state.position_rad[6] = -0.40;
    motor_state.velocity_rad_s[6] = -2.0;

    inference::PolicyAction last_action{};
    inference::PolicyObservationTerms terms;
    std::string error;
    expect(builder.build(motor_state,
                         make_imu_state(),
                         {0.0, 0.0, 0.0},
                         last_action,
                         terms,
                         error),
           "build should produce joint terms: " + error);

    expect_near(terms.joint_pos_rel[0], 0.50F, "motor 0 should map to model 0 position");
    expect_near(terms.joint_vel_rel[0], 0.10F, "motor 0 velocity should scale");
    expect_near(terms.joint_pos_rel[1], 0.40F, "motor 6 direction should map to model 1 position");
    expect_near(terms.joint_vel_rel[1], 0.20F, "motor 6 direction should map to model 1 velocity");

    builder.reset_runtime_state();
    expect(builder.build(motor_state,
                         make_imu_state(),
                         {0.0, 0.0, 0.0},
                         last_action,
                         terms,
                         error),
           "build should produce joint terms after reset: " + error);
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

void test_ankle_velocity_uses_jacobian()
{
    inference::robot_detail::ObservationBuilder builder(make_mapping(),
                                                        make_policy_config());
    inference::MotorStateSnapshot motor_state = make_motor_state();

    constexpr double pitch = -0.05;
    constexpr double roll = 0.04;
    const ankle_motor_ik::MotorAngles motors =
        ankle_motor_ik::solve(roll, pitch);
    expect(motors.reachable(), "left ankle IK should solve the test pose");

    constexpr int upper_motor_index = 4;
    constexpr int lower_motor_index = 5;
    constexpr int pitch_model_index = 8;
    constexpr int roll_model_index = 10;
    constexpr double upper_motor_velocity = 0.25;
    constexpr double lower_motor_velocity = -0.15;

    const int upper_direction = policy_example_config().motor_to_model_direction[upper_motor_index];
    const int lower_direction = policy_example_config().motor_to_model_direction[lower_motor_index];

    motor_state.position_rad[upper_motor_index] = upper_direction * motors.motor1;
    motor_state.position_rad[lower_motor_index] = lower_direction * motors.motor2;
    motor_state.velocity_rad_s[upper_motor_index] = upper_direction * upper_motor_velocity;
    motor_state.velocity_rad_s[lower_motor_index] = lower_direction * lower_motor_velocity;

    ankle_motor_jacobian::Result jacobian;
    std::string jacobian_error;
    expect(ankle_motor_jacobian::solve(pitch,
                                       roll,
                                       motors.motor1,
                                       motors.motor2,
                                       jacobian,
                                       jacobian_error),
           "expected jacobian should solve: " + jacobian_error);
    const double expected_pitch_velocity =
        jacobian.virtual_from_motor[0][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[0][1] * lower_motor_velocity;
    const double expected_roll_velocity =
        jacobian.virtual_from_motor[1][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[1][1] * lower_motor_velocity;

    inference::PolicyAction last_action{};
    inference::PolicyObservationTerms terms;
    std::string error;
    expect(builder.build(motor_state,
                         make_imu_state(),
                         {0.0, 0.0, 0.0},
                         last_action,
                         terms,
                         error),
           "build should produce ankle jacobian velocity terms: " + error);

    expect_near(terms.joint_pos_rel[pitch_model_index],
                static_cast<float>(pitch),
                "left ankle pitch position should come from FK");
    expect_near(terms.joint_pos_rel[roll_model_index],
                static_cast<float>(roll),
                "left ankle roll position should come from FK");
    expect_near(terms.joint_vel_rel[pitch_model_index],
                static_cast<float>(expected_pitch_velocity * 0.1),
                "left ankle pitch velocity should use jacobian");
    expect_near(terms.joint_vel_rel[roll_model_index],
                static_cast<float>(expected_roll_velocity * 0.1),
                "left ankle roll velocity should use jacobian");
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
    test_ankle_velocity_uses_jacobian();
    test_build_rejects_invalid_inputs();

    std::cout << "observation_builder_test passed\n";
    return 0;
}
