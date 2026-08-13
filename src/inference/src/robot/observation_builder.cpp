#include "robot/observation_builder.hpp"

#include "kinematics/ankle_motor_ik.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

namespace inference::robot_detail {
namespace {

bool index_in_range(int index, int count)
{
    return index >= 0 && index < count;
}

bool finite_array3(const std::array<double, 3>& values)
{
    return std::isfinite(values[0]) &&
           std::isfinite(values[1]) &&
           std::isfinite(values[2]);
}

bool finite_vector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

}  // namespace

ObservationBuilder::ObservationBuilder(std::shared_ptr<const JointMapping> mapping,
                                       PolicyConfig policy_config)
    : mapping_(std::move(mapping)),
      policy_config_(std::move(policy_config))
{
    reset_runtime_state();
}

void ObservationBuilder::AnkleFkState::reset(double roll, double pitch)
{
    solver.reset(roll, pitch);
}

void ObservationBuilder::reset_ankle_state(const AnkleParallelMap& ankle_map,
                                                 AnkleFkState&     state)
{
    double pitch = 0.0;
    double roll = 0.0;
    if (policy_config_.stand_pose_rad.size() == kDof &&
        index_in_range(ankle_map.model_pitch_dof, static_cast<int>(kDof)) &&
        index_in_range(ankle_map.model_roll_dof, static_cast<int>(kDof))) {
        pitch = policy_config_.stand_pose_rad[
            static_cast<std::size_t>(ankle_map.model_pitch_dof)];
        roll = policy_config_.stand_pose_rad[
            static_cast<std::size_t>(ankle_map.model_roll_dof)];
    }
    state.reset(roll, pitch);
}

void ObservationBuilder::reset_runtime_state()
{
    if (mapping_ && mapping_->configured()) {
        reset_ankle_state(mapping_->left_ankle(), left_ankle_fk_);
        reset_ankle_state(mapping_->right_ankle(), right_ankle_fk_);
    } else {
        left_ankle_fk_.reset();
        right_ankle_fk_.reset();
    }
    ready_ = false;
    last_time_ = {};
}

bool ObservationBuilder::build(
    const MotorStateSnapshot& motor_state,
    const ImuStateSnapshot& imu_state,
    const std::array<double, 3>& target_velocity,
    const PolicyAction& last_action,
    PolicyObservationTerms& terms,
    std::string& error)
{
    if (motor_state.position_rad.size() != kDof ||
        motor_state.velocity_rad_s.size() != kDof) {
        error = "motor state position/velocity size mismatch";
        return false;
    }
    if (!imu_state.ahrs_ready) {
        error = "AHRS data is not ready";
        return false;
    }
    if (!imu_state.projected_gravity_valid) {
        error = "projected gravity is invalid";
        return false;
    }
    if (!finite_array3(target_velocity)) {
        error = "velocity command is not finite";
        return false;
    }
    if (!finite_vector(motor_state.position_rad) ||
        !finite_vector(motor_state.velocity_rad_s) ||
        !finite_array3(imu_state.body_ang_vel) ||
        !finite_array3(imu_state.projected_gravity)) {
        error = "observation source value is not finite";
        return false;
    }

    PolicyObservationTerms next_terms;
    next_terms.velocity_commands = {
        static_cast<float>(target_velocity[0] * policy_config_.command_scale[0]),
        static_cast<float>(target_velocity[1] * policy_config_.command_scale[1]),
        static_cast<float>(target_velocity[2] * policy_config_.command_scale[2])
    };
    for (int i = 0; i < 3; ++i) {
        next_terms.base_ang_vel[i] = static_cast<float>(
            imu_state.body_ang_vel[i] * policy_config_.body_ang_vel_scale[i]);
        next_terms.projected_gravity[i] =
            static_cast<float>(imu_state.projected_gravity[i]);
    }
    next_terms.last_action = last_action;

    MotorStateArray q_motor_rad{};
    MotorStateArray dq_motor_rad_s{};
    std::copy(motor_state.position_rad.begin(),
              motor_state.position_rad.end(),
              q_motor_rad.begin());
    std::copy(motor_state.velocity_rad_s.begin(),
              motor_state.velocity_rad_s.end(),
              dq_motor_rad_s.begin());

    if (!build_joint_terms(q_motor_rad,
                           dq_motor_rad_s,
                           std::chrono::steady_clock::now(),
                           next_terms.joint_pos_rel,
                           next_terms.joint_vel_rel,
                           error)) {
        return false;
    }

    terms = next_terms;
    error.clear();
    return true;
}

bool ObservationBuilder::build_joint_terms(
    const MotorStateArray& q_motor_rad,
    const MotorStateArray& dq_motor_rad_s,
    std::chrono::steady_clock::time_point now,
    JointTermArray& joint_pos_rel,
    JointTermArray& joint_vel_rel,
    std::string& error)
{
    if (!mapping_ || !mapping_->configured()) {
        error = "joint mapping is not configured";
        return false;
    }
    if (mapping_->dof_count() != static_cast<int>(kDof)) {
        error = "joint mapping DOF count does not match policy DOF";
        return false;
    }
    if (policy_config_.stand_pose_rad.size() != kDof ||
        policy_config_.dof_pos_scale.size() != kDof ||
        policy_config_.dof_vel_scale.size() != kDof) {
        error = "policy observation vectors must have one value per DOF";
        return false;
    }

    joint_pos_rel.fill(0.0F);
    joint_vel_rel.fill(0.0F);

    for (int model_index = 0; model_index < static_cast<int>(kDof); ++model_index) {
        if (mapping_->is_parallel_model_dof(model_index)) {
            continue;
        }

        const int motor_index = mapping_->direct_motor_for_model_dof(model_index);
        if (!index_in_range(motor_index, static_cast<int>(kDof))) {
            error = "joint mapping missing direct motor for model index " +
                    std::to_string(model_index);
            return false;
        }

        const auto model = static_cast<std::size_t>(model_index);
        const auto motor = static_cast<std::size_t>(motor_index);
        const double direction = static_cast<double>(
            mapping_->direction_for_motor(motor_index));
        const double q_model = direction * q_motor_rad[motor];
        const double dq_model = direction * dq_motor_rad_s[motor];

        joint_pos_rel[model] =
            static_cast<float>((q_model - policy_config_.stand_pose_rad[model]) *
                               policy_config_.dof_pos_scale[model]);
        joint_vel_rel[model] =
            static_cast<float>(dq_model * policy_config_.dof_vel_scale[model]);
    }

    const double dt = ready_
        ? std::chrono::duration<double>(now - last_time_).count()
        : 0.0;
    const bool has_valid_dt = ready_ && std::isfinite(dt) && dt > 0.0;

    apply_ankle_fk_observation(q_motor_rad,
                               dt,
                               has_valid_dt,
                               mapping_->left_ankle(),
                               left_ankle_fk_,
                               joint_pos_rel,
                               joint_vel_rel);
    apply_ankle_fk_observation(q_motor_rad,
                               dt,
                               has_valid_dt,
                               mapping_->right_ankle(),
                               right_ankle_fk_,
                               joint_pos_rel,
                               joint_vel_rel);

    ready_ = true;
    last_time_ = now;
    error.clear();
    return true;
}

void ObservationBuilder::apply_ankle_fk_observation(
    const MotorStateArray& q_motor_rad,
    double dt,
    bool has_valid_dt,
    const AnkleParallelMap& ankle_map,
    AnkleFkState& state,
    JointTermArray& joint_pos_rel,
    JointTermArray& joint_vel_rel) const
{
    const int ankle_model_pitch_dof = ankle_map.model_pitch_dof;
    const int ankle_model_roll_dof = ankle_map.model_roll_dof;
    const int upper_motor_direction =
        mapping_->direction_for_motor(ankle_map.upper_motor_index);
    const int lower_motor_direction =
        mapping_->direction_for_motor(ankle_map.lower_motor_index);
    const double upper_motor = upper_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)];
    const double lower_motor = lower_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)];

    const double previous_roll = state.solver.previous_roll();
    const double previous_pitch = state.solver.previous_pitch();
    const ankle_motor_fk::FootAngles foot =
        state.solver.solve(upper_motor, lower_motor);

    const auto pitch_index = static_cast<std::size_t>(ankle_model_pitch_dof);
    const auto roll_index = static_cast<std::size_t>(ankle_model_roll_dof);
    joint_pos_rel[pitch_index] =
        static_cast<float>((foot.pitch - policy_config_.stand_pose_rad[pitch_index]) *
                           policy_config_.dof_pos_scale[pitch_index]);
    joint_pos_rel[roll_index] =
        static_cast<float>((foot.roll - policy_config_.stand_pose_rad[roll_index]) *
                           policy_config_.dof_pos_scale[roll_index]);

    double pitch_velocity = 0.0;
    double roll_velocity  = 0.0;
    if (has_valid_dt) {
        pitch_velocity = ankle_motor_ik::wrap_to_pi(foot.pitch - previous_pitch) / dt;
        roll_velocity  = ankle_motor_ik::wrap_to_pi(foot.roll - previous_roll) / dt;
    }

    joint_vel_rel[pitch_index] =
        static_cast<float>(pitch_velocity * policy_config_.dof_vel_scale[pitch_index]);
    joint_vel_rel[roll_index] =
        static_cast<float>(roll_velocity * policy_config_.dof_vel_scale[roll_index]);
}

}  // namespace inference::robot_detail
