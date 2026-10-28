#include "config/deploy_config.hpp"
#include "robot/action_processor.hpp"
#include "robot/observation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void check_history(bool gait)
{
    using namespace inference;
    RobotInterfaceConfig config;
    std::string error;
    require(load_deploy_config(ROBOT_DEPLOY_CONFIG_PATH, config, error), error);
    config.policy.gait.enabled = gait;
    config.observation_scales.command_scale = {2, 3, 4};
    auto mapping = robot_detail::JointMapping::create(12, config.joint_mapping, error);
    require(mapping != nullptr, error);
    robot_detail::ObservationBuilder builder(mapping, config.observation_scales,
                                              config.action.default_joint_pos_rad,
                                              config.policy);
    robot_detail::ActionProcessor actions(mapping, config.action, config.ankle_motor_limits,
                                          config.motor.mit_kp, config.motor.mit_kd,
                                          config.ankle_torque);
    PolicyAction committed_action{};
    // An independent frame table checks term-major flattening and oldest-first order.
    std::vector<std::vector<std::vector<float>>> frames;
    for (int frame = 0; frame < 40; ++frame) {
        auto target = config.action.default_joint_pos_rad;
        for (std::size_t i = 0; i < target.size(); ++i)
            target[i] += 0.02 * std::sin(frame * 0.1 + i);
        MotorStateSnapshot motor;
        require(actions.build_motor_targets(target, motor.position_rad, error), error);
        AhrsStateSnapshot ahrs;
        ahrs.projected_gravity_valid = true;
        ahrs.body_ang_vel = {frame * 0.1, -0.5, 0.25};
        ahrs.projected_gravity = {0.1, 0.2, -0.9};
        std::array<double, 3> command = frame == 0 ? std::array<double, 3>{0, 0, 0}
            : frame == 1 ? std::array<double, 3>{0.05, 0, 0}
                         : std::array<double, 3>{0.3, -0.1, 0.2};
        PolicyObservation observation{};
        require(builder.build(motor, ahrs, command, observation, error), error);
        require(builder.frame_index() == static_cast<std::uint64_t>(frame), "frame advanced in build");
        std::vector<std::vector<float>> terms(3);
        for (std::size_t i = 0; i < 3; ++i) {
            terms[0].push_back(ahrs.body_ang_vel[i] * config.observation_scales.body_ang_vel_scale[i]);
            terms[1].push_back(ahrs.projected_gravity[i]);
            terms[2].push_back(command[i] * config.observation_scales.command_scale[i]);
        }
        if (gait) {
            const double phase = 6.28318530717958647692 * std::fmod(frame * 0.02, 0.74) / 0.74;
            const double gate = frame == 0 ? 0 : frame == 1 ? 0.5 : 1;
            terms.push_back({float(std::sin(phase) * gate), float(std::cos(phase) * gate)});
        }
        std::vector<float> positions;
        for (std::size_t i = 0; i < target.size(); ++i)
            positions.push_back((target[i] - config.action.default_joint_pos_rad[i]) *
                                config.observation_scales.dof_pos_scale[i]);
        terms.push_back(positions);
        terms.push_back(std::vector<float>(12, 0));
        terms.emplace_back(committed_action.begin(), committed_action.end());
        frames.push_back(terms);
        std::size_t index = 0;
        for (std::size_t term = 0; term < terms.size(); ++term) {
            for (int age = 14; age >= 0; --age) {
                for (float expected : frames[std::max(0, frame - age)][term]) {
                    require(std::abs(observation[index] - expected) < 1e-4F,
                            "observation mismatch at frame " + std::to_string(frame) +
                            " index " + std::to_string(index));
                    ++index;
                }
            }
        }
        require(index == policy_observation::observation_size(gait), "observation dimension mismatch");
        for (; index < observation.size(); ++index)
            require(observation[index] == 0, "unused observation capacity changed");
        // Skip committing some frames, as the age admission guard does for drops.
        if (frame % 3) {
            PolicyAction raw_action{};
            for (std::size_t i = 0; i < raw_action.size(); ++i) {
                raw_action[i] = (i % 2 ? -2.F : 2.F) + frame * 0.01F;
                committed_action[i] = std::clamp(raw_action[i], -1.F, 1.F);
            }
            builder.commit_policy_action(raw_action);
        }
        builder.advance_frame();
    }
    builder.reset_runtime_state();
    require(builder.frame_index() == 0, "reset retained frame index");
    MotorStateSnapshot motor;
    require(actions.build_motor_targets(config.action.default_joint_pos_rad, motor.position_rad, error), error);
    AhrsStateSnapshot ahrs;
    PolicyObservation observation{};
    require(!builder.build(motor, ahrs, {0, 0, 0}, observation, error), "invalid gravity accepted");
    ahrs.projected_gravity_valid = true;
    require(!builder.build(motor, ahrs, {std::numeric_limits<double>::infinity(), 0, 0},
                           observation, error), "invalid velocity accepted");
    require(builder.build(motor, ahrs, {0, 0, 0}, observation, error), error);
    const auto action_offset = policy_observation::observation_size(gait) - 15 * 12;
    for (std::size_t i = action_offset; i < action_offset + 15 * 12; ++i)
        require(observation[i] == 0, "reset retained action history");
}

}  // namespace

int main()
{
    try {
        check_history(false);
        check_history(true);
        std::cout << "observation_builder_test passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
