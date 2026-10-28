#include "robot/observation_builder.hpp"

#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <utility>

namespace inference::robot_detail {

using robot_base::finite_array;

namespace {

struct ObservationTerms {
    std::array<float, policy_observation::kBaseAngVelSize> base_ang_vel{};
    std::array<float, policy_observation::kProjectedGravitySize> projected_gravity{};
    std::array<float, policy_observation::kVelocityCommandsSize> velocity_commands{};
    std::array<float, policy_observation::kJointPosRelSize> joint_pos_rel{};
    std::array<float, policy_observation::kJointVelRelSize> joint_vel_rel{};
    PolicyAction last_action{};
};

std::array<float, 2> gated_gait_phase_observation(
    const ObservationTerms& terms,
    const PolicyRuntimeConfig& policy_config,
    std::uint64_t episode_length)
{
    const double command_norm = std::sqrt(
        static_cast<double>(terms.velocity_commands[0]) * terms.velocity_commands[0] +
        static_cast<double>(terms.velocity_commands[1]) * terms.velocity_commands[1] +
        static_cast<double>(terms.velocity_commands[2]) * terms.velocity_commands[2]);

    double gate = std::clamp(
        (command_norm - policy_config.gait.stand_threshold) /
            (policy_config.gait.move_threshold -
             policy_config.gait.stand_threshold),
        0.0,
        1.0);
    gate = gate * gate * (3.0 - 2.0 * gate);

    constexpr double kTwoPi = 6.28318530717958647692;
    const double global_phase = std::fmod(
        static_cast<double>(episode_length) * policy_config.step_dt,
        policy_config.gait.period) / policy_config.gait.period;
    const std::array<float, 2> phase{
        static_cast<float>(std::sin(global_phase * kTwoPi)),
        static_cast<float>(std::cos(global_phase * kTwoPi))
    };
    return {
        static_cast<float>(phase[0] * gate),
        static_cast<float>(phase[1] * gate)
    };
}

template <std::size_t TermSize>
void update_term_history(PolicyObservation& history,
                         std::size_t offset,
                         const std::array<float, TermSize>& current_term,
                         bool history_ready)
{
    const auto begin = history.begin() + offset;
    const auto end = begin + policy_observation::kFrameStack * TermSize;
    if (history_ready) {
        std::copy(begin + TermSize, end, begin);
        std::copy(current_term.begin(), current_term.end(), end - TermSize);
    } else {
        for (auto frame = begin; frame != end; frame += TermSize) {
            std::copy(current_term.begin(), current_term.end(), frame);
        }
    }
}

}  // namespace

ObservationBuilder::ObservationBuilder(std::shared_ptr<const JointMapping> mapping,
                                       ObservationScaleConfig scales,
                                       std::array<double, policy_observation::kDof> default_joint_pos_rad,
                                       PolicyRuntimeConfig policy_config)
    : mapping_(std::move(mapping)),
      scales_(std::move(scales)),
      default_joint_pos_rad_(default_joint_pos_rad),
      policy_config_(std::move(policy_config))
{
    reset_runtime_state();
}


void ObservationBuilder::reset_runtime_state()
{
    reset_ankle_state(mapping_->left_ankle(),  left_ankle_fk_);
    reset_ankle_state(mapping_->right_ankle(), right_ankle_fk_);
    last_action_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    frame_index_ = 0;
}

void ObservationBuilder::commit_policy_action(const PolicyAction& raw_action) noexcept
{
    for (std::size_t i = 0; i < raw_action.size(); ++i) {
        last_action_[i] = std::clamp(raw_action[i], -1.0F, 1.0F);
    }
}

void ObservationBuilder::advance_frame() noexcept
{
    ++frame_index_;
}

// 从default_joint_pos_rad_中获取roll和pitch值，调用ankle_fk的reset函数，重置ankle_fk
void ObservationBuilder::reset_ankle_state(const AnkleParallelMap& ankle_map,
                                                 ankle_motor_fk::Solver& state)
{
    const double pitch = default_joint_pos_rad_[static_cast<std::size_t>(ankle_map.model_pitch_dof)];
    const double roll  = default_joint_pos_rad_[static_cast<std::size_t>(ankle_map.model_roll_dof)];
    state.reset(roll, pitch);
}


bool ObservationBuilder::build(
    const MotorStateSnapshot&    motor_state,       // 电机快照
    const AhrsStateSnapshot&     ahrs_state,        // AHRS快照
    const std::array<double, 3>& target_velocity,   // 目标速度
    PolicyObservation&           observation,       // 完整策略观测历史
    std::string& error)
{
    if (!ahrs_state.projected_gravity_valid) {
        error = "projected gravity is invalid";
        return false;
    }
    if (!finite_array(target_velocity)) {
        error = "velocity command is not finite";
        return false;
    }
    if (!finite_array(motor_state.position_rad) || !finite_array(motor_state.velocity_rad_s) ||
        !finite_array(ahrs_state.body_ang_vel)   || !finite_array(ahrs_state.projected_gravity)) {
        error = "observation source value is not finite";
        return false;
    }

    // 使用当前数据构建下一观测帧
    ObservationTerms current_terms;
    current_terms.velocity_commands = {
        static_cast<float>(target_velocity[0] * scales_.command_scale[0]),
        static_cast<float>(target_velocity[1] * scales_.command_scale[1]),
        static_cast<float>(target_velocity[2] * scales_.command_scale[2])
    };
    for (int i = 0; i < 3; ++i) {
        current_terms.base_ang_vel[i] = static_cast<float>(
            ahrs_state.body_ang_vel[i] * scales_.body_ang_vel_scale[i]);
        current_terms.projected_gravity[i] =
            static_cast<float>(ahrs_state.projected_gravity[i]);
    }
    
    current_terms.last_action = last_action_;


    if (!fill_joint_terms(motor_state.position_rad,
                          motor_state.velocity_rad_s,
                          current_terms.joint_pos_rel,
                          current_terms.joint_vel_rel,
                          error)) {
        return false;
    }

    constexpr std::size_t kBaseAngVelOffset = 0;
    constexpr std::size_t kBaseAngVelHistorySize =
        policy_observation::kFrameStack * policy_observation::kBaseAngVelSize;
    constexpr std::size_t kProjectedGravityOffset =
        kBaseAngVelOffset + kBaseAngVelHistorySize;
    constexpr std::size_t kProjectedGravityHistorySize =
        policy_observation::kFrameStack * policy_observation::kProjectedGravitySize;
    constexpr std::size_t kVelocityCommandsOffset =
        kProjectedGravityOffset + kProjectedGravityHistorySize;
    constexpr std::size_t kVelocityCommandsHistorySize =
        policy_observation::kFrameStack * policy_observation::kVelocityCommandsSize;
    constexpr std::size_t kGaitPhaseOffset =
        kVelocityCommandsOffset + kVelocityCommandsHistorySize;
    const bool gait_phase_enabled = policy_config_.gait.enabled;
    const std::size_t gait_phase_history_size =
        gait_phase_enabled
            ? policy_observation::kFrameStack * policy_observation::kGaitPhaseSize
            : 0;
    const std::size_t joint_pos_rel_offset =
        kGaitPhaseOffset + gait_phase_history_size;
    constexpr std::size_t kJointPosRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointPosRelSize;
    const std::size_t joint_vel_rel_offset =
        joint_pos_rel_offset + kJointPosRelHistorySize;
    constexpr std::size_t kJointVelRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointVelRelSize;
    const std::size_t last_action_offset =
        joint_vel_rel_offset + kJointVelRelHistorySize;

    update_term_history(observation_history_, kBaseAngVelOffset,
                        current_terms.base_ang_vel, observation_history_ready_);
    update_term_history(observation_history_, kProjectedGravityOffset,
                        current_terms.projected_gravity, observation_history_ready_);
    update_term_history(observation_history_, kVelocityCommandsOffset,
                        current_terms.velocity_commands, observation_history_ready_);
    if (gait_phase_enabled) {
        const auto gait_phase = gated_gait_phase_observation(
            current_terms, policy_config_, frame_index_);
        update_term_history(observation_history_, kGaitPhaseOffset,
                            gait_phase, observation_history_ready_);
    }
    update_term_history(observation_history_, joint_pos_rel_offset,
                        current_terms.joint_pos_rel, observation_history_ready_);
    update_term_history(observation_history_, joint_vel_rel_offset,
                        current_terms.joint_vel_rel, observation_history_ready_);
    update_term_history(observation_history_, last_action_offset,
                        current_terms.last_action, observation_history_ready_);
    observation_history_ready_ = true;

    observation = observation_history_;
    error.clear();
    return true;
}

bool ObservationBuilder::fill_joint_terms(
    const MotorStateArray& q_motor_rad,
    const MotorStateArray& dq_motor_rad_s, // 输入物理顺序的电机位置和速度
    JointTermArray& joint_pos_rel,         
    JointTermArray& joint_vel_rel,         // 输出模型顺序的关节的相对偏移和速度
    std::string& error)
{
    joint_pos_rel.fill(0.0F);
    joint_vel_rel.fill(0.0F);

    // 按照模型顺序遍历关节
    for (int model_index = 0; model_index < kDof; ++model_index) {
        // 如果是脚踝模型的关节，则跳过
        if (mapping_->is_parallel_model_dof(model_index)) {
            continue;
        }

        // 得到对应电机索引
        const int motor_index = mapping_->direct_motor_for_model_dof(model_index);
        // 根据方向进行转换
        const double direction = static_cast<double>(mapping_->direction_for_motor(motor_index));
        const double q_model   = direction * q_motor_rad[motor_index];
        const double dq_model  = direction * dq_motor_rad_s[motor_index];

        //  计算关节的相对偏移和速度，并进行缩放
        joint_pos_rel[model_index] =
            static_cast<float>((q_model - default_joint_pos_rad_[model_index]) *
                               scales_.dof_pos_scale[model_index]);
        joint_vel_rel[model_index] =
            static_cast<float>(dq_model * scales_.dof_vel_scale[model_index]);
    }

    // 针对脚踝模型的关节，使用FK计算位置，并用解析Jacobian把电机速度映射为虚拟关节速度。
    if (!fill_ankle_fk_joint_terms(q_motor_rad,
                                   dq_motor_rad_s,
                                   mapping_->left_ankle(),
                                   left_ankle_fk_,
                                   joint_pos_rel,
                                   joint_vel_rel,
                                   error)) {
        return false;
    }
    if (!fill_ankle_fk_joint_terms(q_motor_rad,
                                   dq_motor_rad_s,
                                   mapping_->right_ankle(),
                                   right_ankle_fk_,
                                   joint_pos_rel,
                                   joint_vel_rel,
                                   error)) {
        return false;
    }

    error.clear();
    return true;
}

bool ObservationBuilder::fill_ankle_fk_joint_terms(
    const MotorStateArray& q_motor_rad,
    const MotorStateArray& dq_motor_rad_s,
    const AnkleParallelMap& ankle_map,
    ankle_motor_fk::Solver& state,
    JointTermArray& joint_pos_rel,
    JointTermArray& joint_vel_rel,
    std::string& error) const
{
    // 获取脚踝模型的pitch和roll关节索引
    const int ankle_model_pitch_dof = ankle_map.model_pitch_dof;
    const int ankle_model_roll_dof  = ankle_map.model_roll_dof;

    // 和运动学解算器中电机的旋转方向对齐
    const int upper_motor_direction =
        mapping_->direction_for_motor(ankle_map.upper_motor_index);
    const int lower_motor_direction =
        mapping_->direction_for_motor(ankle_map.lower_motor_index);

    
    const double upper_motor = upper_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)];
    const double lower_motor = lower_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)];
    const double upper_motor_velocity = upper_motor_direction *
        dq_motor_rad_s[static_cast<std::size_t>(ankle_map.upper_motor_index)];
    const double lower_motor_velocity = lower_motor_direction *
        dq_motor_rad_s[static_cast<std::size_t>(ankle_map.lower_motor_index)];

    const ankle_motor_fk::FootAngles foot =
        state.solve(upper_motor, lower_motor);
    if (!foot.reachable ||
        !std::isfinite(foot.pitch) ||
        !std::isfinite(foot.roll)) {
        error = "ankle FK failed while building observation";
        return false;
    }

    ankle_motor_jacobian::Result jacobian;
    std::string jacobian_error;
    if (!ankle_motor_jacobian::solve(foot.pitch,
                                     foot.roll,
                                     upper_motor,
                                     lower_motor,
                                     jacobian,
                                     jacobian_error)) {
        error = "failed to build ankle jacobian: " + jacobian_error;
        return false;
    }

    const auto pitch_index = static_cast<std::size_t>(ankle_model_pitch_dof);
    const auto roll_index  = static_cast<std::size_t>(ankle_model_roll_dof);


    // 计算脚踝关节的相对偏移并缩放
    joint_pos_rel[pitch_index] =
        static_cast<float>((foot.pitch - default_joint_pos_rad_[pitch_index]) *
                           scales_.dof_pos_scale[pitch_index]);
    joint_pos_rel[roll_index] =
        static_cast<float>((foot.roll - default_joint_pos_rad_[roll_index]) *
                           scales_.dof_pos_scale[roll_index]);


    // 使用 q_v_dot = J q_m_dot 计算脚踝虚拟关节速度，顺序为 [pitch, roll]。
    const double pitch_velocity =
        jacobian.virtual_from_motor[0][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[0][1] * lower_motor_velocity;
    const double roll_velocity =
        jacobian.virtual_from_motor[1][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[1][1] * lower_motor_velocity;
    if (!std::isfinite(pitch_velocity) || !std::isfinite(roll_velocity)) {
        error = "ankle jacobian velocity is not finite";
        return false;
    }
    // 缩放脚踝关节的速度
    joint_vel_rel[pitch_index] =
        static_cast<float>(pitch_velocity * scales_.dof_vel_scale[pitch_index]);
    joint_vel_rel[roll_index] =
        static_cast<float>(roll_velocity * scales_.dof_vel_scale[roll_index]);

    error.clear();
    return true;
}

}  // namespace inference::robot_detail
