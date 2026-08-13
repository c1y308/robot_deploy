#include "robot/action_processor.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace inference::robot_detail {
namespace {

bool index_in_range(int index, int count)
{
    return index >= 0 && index < count;
}

}  // namespace

ActionProcessor::ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                                 PolicyConfig policy_config)
    : mapping_(std::move(mapping)),
      policy_config_(std::move(policy_config))
{
}

void ActionProcessor::AnkleIkState::reset()
{
    solver.reset();
    last_upper_motor = 0.0;
    last_lower_motor = 0.0;
    solved = false;
}

void ActionProcessor::reset_runtime_state()
{
    left_ankle_ik_.reset();
    right_ankle_ik_.reset();
}

int ActionProcessor::dof_count() const noexcept
{
    return mapping_ ? mapping_->dof_count() : 0;
}

bool ActionProcessor::has_relative_limits() const
{
    const auto count = static_cast<std::size_t>(dof_count());
    return policy_config_.stand_pose_rad.size() == count &&
           policy_config_.joint_min_rad.size() == count &&
           policy_config_.joint_max_rad.size() == count;
}

bool ActionProcessor::apply_ankle_ik(
    const std::vector<double>& target_q_model_rad,
    std::vector<double>& target_motor_rad,
    const AnkleParallelMap& ankle_map,
    AnkleIkState& state,
    std::string& error)
{
    if (!mapping_) {
        error = "joint mapping is not configured";
        return false;
    }

    const int count = dof_count();
    if (!index_in_range(ankle_map.model_pitch_dof, count) ||
        !index_in_range(ankle_map.model_roll_dof, count) ||
        !index_in_range(ankle_map.upper_motor_index, count) ||
        !index_in_range(ankle_map.lower_motor_index, count)) {
        error = "ankle map contains an out-of-range index";
        return false;
    }

    double pitch = target_q_model_rad[static_cast<std::size_t>(ankle_map.model_pitch_dof)];
    double roll = target_q_model_rad[static_cast<std::size_t>(ankle_map.model_roll_dof)];
    if (has_relative_limits()) {
        const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
        const auto roll_index = static_cast<std::size_t>(ankle_map.model_roll_dof);
        const double pitch_lo = policy_config_.stand_pose_rad[pitch_index] +
                                policy_config_.joint_min_rad[pitch_index];
        const double pitch_hi = policy_config_.stand_pose_rad[pitch_index] +
                                policy_config_.joint_max_rad[pitch_index];
        const double roll_lo = policy_config_.stand_pose_rad[roll_index] +
                               policy_config_.joint_min_rad[roll_index];
        const double roll_hi = policy_config_.stand_pose_rad[roll_index] +
                               policy_config_.joint_max_rad[roll_index];
        pitch = std::max(pitch_lo, std::min(pitch_hi, pitch));
        roll = std::max(roll_lo, std::min(roll_hi, roll));
    }

    const ankle_motor_ik::MotorAngles result = state.solver.solve(roll, pitch);

    double upper_motor = 0.0;
    double lower_motor = 0.0;
    if (result.reachable()) {
        upper_motor = result.motor1;
        lower_motor = result.motor2;
        state.last_upper_motor = upper_motor;
        state.last_lower_motor = lower_motor;
        state.solved = true;
    } else if (state.solved) {
        upper_motor = state.last_upper_motor;
        lower_motor = state.last_lower_motor;
    }

    const int upper_motor_direction =
        mapping_->direction_for_motor(ankle_map.upper_motor_index);
    const int lower_motor_direction =
        mapping_->direction_for_motor(ankle_map.lower_motor_index);
    target_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)] =
        upper_motor_direction * upper_motor;
    target_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)] =
        lower_motor_direction * lower_motor;
    error.clear();
    return true;
}

bool ActionProcessor::build_motor_targets(
    const std::vector<double>& target_q_model_rad,
    std::vector<double>& target_motor_rad,
    std::string& error)
{
    if (!mapping_) {
        error = "joint mapping is not configured";
        return false;
    }

    const int count = dof_count();
    if (static_cast<int>(target_q_model_rad.size()) != count) {
        error = "target size mismatch";
        return false;
    }

    target_motor_rad.assign(static_cast<std::size_t>(count), 0.0);
    const bool apply_relative_limits = has_relative_limits();

    for (int model_index = 0; model_index < count; ++model_index) {
        if (mapping_->is_parallel_model_dof(model_index)) {
            continue;
        }

        const int motor_index = mapping_->direct_motor_for_model_dof(model_index);
        if (!index_in_range(motor_index, count)) {
            error = "joint mapping missing direct motor for model index " +
                    std::to_string(model_index);
            return false;
        }

        double q = target_q_model_rad[static_cast<std::size_t>(model_index)];
        if (apply_relative_limits) {
            const auto index = static_cast<std::size_t>(model_index);
            const double lower_limit =
                policy_config_.stand_pose_rad[index] + policy_config_.joint_min_rad[index];
            const double upper_limit =
                policy_config_.stand_pose_rad[index] + policy_config_.joint_max_rad[index];
            q = std::max(lower_limit, std::min(upper_limit, q));
        }

        target_motor_rad[static_cast<std::size_t>(motor_index)] =
            mapping_->direction_for_motor(motor_index) * q;
    }

    if (!apply_ankle_ik(target_q_model_rad,
                        target_motor_rad,
                        mapping_->left_ankle(),
                        left_ankle_ik_,
                        error)) {
        return false;
    }
    if (!apply_ankle_ik(target_q_model_rad,
                        target_motor_rad,
                        mapping_->right_ankle(),
                        right_ankle_ik_,
                        error)) {
        return false;
    }

    error.clear();
    return true;
}

bool ActionProcessor::build_reset_start_model_pose(
    const std::vector<double>& q0_motor,
    const std::vector<double>& target_model,
    std::vector<double>& q0_model,
    std::string& error) const
{
    if (!mapping_) {
        error = "joint mapping is not configured";
        return false;
    }

    const int count = dof_count();
    if (static_cast<int>(q0_motor.size()) != count ||
        static_cast<int>(target_model.size()) != count) {
        error = "reset pose size mismatch";
        return false;
    }

    q0_model.assign(static_cast<std::size_t>(count), 0.0);
    for (int model_index = 0; model_index < count; ++model_index) {
        if (mapping_->is_parallel_model_dof(model_index)) {
            continue;
        }

        const int motor_index = mapping_->direct_motor_for_model_dof(model_index);
        if (!index_in_range(motor_index, count)) {
            error = "joint mapping missing direct motor for model index " +
                    std::to_string(model_index);
            return false;
        }
        q0_model[static_cast<std::size_t>(model_index)] =
            mapping_->direction_for_motor(motor_index) *
            q0_motor[static_cast<std::size_t>(motor_index)];
    }

    auto fill_ankle_q0_model = [&](const AnkleParallelMap& ankle_map) {
        const double upper_motor =
            mapping_->direction_for_motor(ankle_map.upper_motor_index) *
            q0_motor[static_cast<std::size_t>(ankle_map.upper_motor_index)];
        const double lower_motor =
            mapping_->direction_for_motor(ankle_map.lower_motor_index) *
            q0_motor[static_cast<std::size_t>(ankle_map.lower_motor_index)];

        const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
        const auto roll_index = static_cast<std::size_t>(ankle_map.model_roll_dof);
        const double initial_pitch = target_model[pitch_index];
        const double initial_roll = target_model[roll_index];
        const ankle_motor_fk::FootAngles foot =
            ankle_motor_fk::solve(upper_motor, lower_motor, initial_roll, initial_pitch);

        if (foot.reachable) {
            q0_model[pitch_index] = foot.pitch;
            q0_model[roll_index] = foot.roll;
        } else {
            q0_model[pitch_index] = initial_pitch;
            q0_model[roll_index] = initial_roll;
        }
        return true;
    };

    fill_ankle_q0_model(mapping_->left_ankle());
    fill_ankle_q0_model(mapping_->right_ankle());
    error.clear();
    return true;
}

}  // namespace inference::robot_detail
