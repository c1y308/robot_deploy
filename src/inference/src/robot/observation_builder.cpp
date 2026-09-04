#include "robot/observation_builder.hpp"

#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

namespace inference::robot_detail {

using robot_base::finite_array;
using robot_base::finite_vector;
using robot_base::index_in_range;

namespace {

struct ObservationTerms {
    std::array<float, policy_observation::kBaseAngVelSize> base_ang_vel{};
    std::array<float, policy_observation::kProjectedGravitySize> projected_gravity{};
    std::array<float, policy_observation::kVelocityCommandsSize> velocity_commands{};
    std::array<float, policy_observation::kJointPosRelSize> joint_pos_rel{};
    std::array<float, policy_observation::kJointVelRelSize> joint_vel_rel{};
    PolicyAction last_action{};
};

std::array<float, 2> gait_phase_observation(std::uint64_t episode_length,
                                            double step_dt,
                                            double period)
{
    constexpr double kTwoPi = 6.28318530717958647692;
    const double global_phase =
        std::fmod(static_cast<double>(episode_length) * step_dt, period) / period;
    return {
        static_cast<float>(std::sin(global_phase * kTwoPi)),
        static_cast<float>(std::cos(global_phase * kTwoPi))
    };
}

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

    const std::array<float, 2> phase =
        gait_phase_observation(episode_length,
                               policy_config.step_dt,
                               policy_config.gait.period);
    return {
        static_cast<float>(phase[0] * gate),
        static_cast<float>(phase[1] * gate)
    };
}

template <std::size_t TermSize, std::size_t ObservationSize>
void fill_term_history(std::array<float, ObservationSize>& history,
                       std::size_t offset,
                       std::size_t frame_stack,
                       const std::array<float, TermSize>& current_term)
{
    for (std::size_t frame = 0; frame < frame_stack; ++frame) {
        std::copy(current_term.begin(),
                  current_term.end(),
                  history.begin() + offset + frame * TermSize);
    }
}

template <std::size_t TermSize, std::size_t ObservationSize>
void append_term_history(std::array<float, ObservationSize>& history,
                         std::size_t offset,
                         std::size_t frame_stack,
                         const std::array<float, TermSize>& current_term)
{
    const std::size_t term_history_size = frame_stack * TermSize;
    std::copy(history.begin() + offset + TermSize,
              history.begin() + offset + term_history_size,
              history.begin() + offset);
    std::copy(current_term.begin(),
              current_term.end(),
              history.begin() + offset + term_history_size - TermSize);
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
    // 如果mapping_存在且已配置，则重置左右ankle_fk的状态
    if (mapping_ && mapping_->configured()) {
        reset_ankle_state(mapping_->left_ankle(),  left_ankle_fk_);
        reset_ankle_state(mapping_->right_ankle(), right_ankle_fk_);
    } 
    // 否则，重置左右ankle_fk的状态为默认值
    else {
        left_ankle_fk_.reset();
        right_ankle_fk_.reset();
    }
    last_action_raw_.fill(0.0F);
    observation_history_.fill(0.0F);
    observation_history_ready_ = false;
    episode_length_ = 0;
    frame_index_ = 0;
}

void ObservationBuilder::commit_policy_action(const PolicyAction& raw_action) noexcept
{
    last_action_raw_ = raw_action;
}

void ObservationBuilder::advance_frame() noexcept
{
    ++frame_index_;
}

void ObservationBuilder::advance_episode() noexcept
{
    ++episode_length_;
}

// 从default_joint_pos_rad_中获取roll和pitch值，调用ankle_fk的reset函数，重置ankle_fk
void ObservationBuilder::reset_ankle_state(const AnkleParallelMap& ankle_map,
                                                 AnkleFkState&     state)
{
    double pitch = 0.0;
    double roll  = 0.0;
    if (index_in_range(ankle_map.model_pitch_dof, static_cast<int>(kDof)) &&
        index_in_range(ankle_map.model_roll_dof, static_cast<int>(kDof)))
    {
        pitch = default_joint_pos_rad_[static_cast<std::size_t>(ankle_map.model_pitch_dof)];
        roll  = default_joint_pos_rad_[static_cast<std::size_t>(ankle_map.model_roll_dof)];
    }
    state.reset(roll, pitch);
}

// fk没有额外操作，直接转发 solver 的 reset 函数
void ObservationBuilder::AnkleFkState::reset(double roll, double pitch)
{
    solver.reset(roll, pitch);
}


bool ObservationBuilder::build(
    const MotorStateSnapshot&    motor_state,       // 电机快照
    const AhrsStateSnapshot&     ahrs_state,        // AHRS快照
    const std::array<double, 3>& target_velocity,   // 目标速度
    PolicyObservation&           observation,       // 完整策略观测历史
    std::string& error)
{
    if (motor_state.position_rad.size() != kDof || motor_state.velocity_rad_s.size() != kDof) {
        error = "motor state position/velocity size mismatch";
        return false;
    }
    if (!ahrs_state.ahrs_ready) {
        error = "AHRS data is not ready";
        return false;
    }
    if (!ahrs_state.projected_gravity_valid) {
        error = "projected gravity is invalid";
        return false;
    }
    if (!finite_array(target_velocity)) {
        error = "velocity command is not finite";
        return false;
    }
    if (!finite_vector(motor_state.position_rad) || !finite_vector(motor_state.velocity_rad_s) ||
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
    
    current_terms.last_action = last_action_raw_;


    MotorStateArray q_motor_rad{};
    MotorStateArray dq_motor_rad_s{};

    std::copy(motor_state.position_rad.begin(),
               motor_state.position_rad.end(),
              q_motor_rad.begin());

    std::copy(motor_state.velocity_rad_s.begin(),
               motor_state.velocity_rad_s.end(),
              dq_motor_rad_s.begin());

    if (!fill_joint_terms(q_motor_rad,
                          dq_motor_rad_s,
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
    constexpr std::size_t kGaitPhaseHistorySize =
        policy_observation::kEnableGaitPhase
            ? policy_observation::kFrameStack * policy_observation::kGaitPhaseSize
            : 0;
    constexpr std::size_t kJointPosRelOffset =
        kGaitPhaseOffset + kGaitPhaseHistorySize;
    constexpr std::size_t kJointPosRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointPosRelSize;
    constexpr std::size_t kJointVelRelOffset =
        kJointPosRelOffset + kJointPosRelHistorySize;
    constexpr std::size_t kJointVelRelHistorySize =
        policy_observation::kFrameStack * policy_observation::kJointVelRelSize;
    constexpr std::size_t kLastActionOffset =
        kJointVelRelOffset + kJointVelRelHistorySize;
    constexpr std::size_t kObservationEnd =
        kLastActionOffset + policy_observation::kFrameStack * policy_observation::kLastActionSize;
    static_assert(kObservationEnd == policy_observation::kObservationSize,
                  "policy observation offsets must cover the configured input size");

    if (!observation_history_ready_) {
        fill_term_history(observation_history_, kBaseAngVelOffset,
                          policy_observation::kFrameStack, current_terms.base_ang_vel);
        fill_term_history(observation_history_, kProjectedGravityOffset,
                          policy_observation::kFrameStack, current_terms.projected_gravity);
        fill_term_history(observation_history_, kVelocityCommandsOffset,
                          policy_observation::kFrameStack, current_terms.velocity_commands);
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gated_gait_phase_observation(current_terms,
                                             policy_config_,
                                             episode_length_);
            fill_term_history(observation_history_, kGaitPhaseOffset,
                              policy_observation::kFrameStack, gait_phase);
        }
        fill_term_history(observation_history_, kJointPosRelOffset,
                          policy_observation::kFrameStack, current_terms.joint_pos_rel);
        fill_term_history(observation_history_, kJointVelRelOffset,
                          policy_observation::kFrameStack, current_terms.joint_vel_rel);
        fill_term_history(observation_history_, kLastActionOffset,
                          policy_observation::kFrameStack, current_terms.last_action);
        observation_history_ready_ = true;
    } else {
        append_term_history(observation_history_, kBaseAngVelOffset,
                            policy_observation::kFrameStack, current_terms.base_ang_vel);
        append_term_history(observation_history_, kProjectedGravityOffset,
                            policy_observation::kFrameStack, current_terms.projected_gravity);
        append_term_history(observation_history_, kVelocityCommandsOffset,
                            policy_observation::kFrameStack, current_terms.velocity_commands);
        if constexpr (policy_observation::kEnableGaitPhase) {
            const std::array<float, 2> gait_phase =
                gated_gait_phase_observation(current_terms,
                                             policy_config_,
                                             episode_length_);
            append_term_history(observation_history_, kGaitPhaseOffset,
                                policy_observation::kFrameStack, gait_phase);
        }
        append_term_history(observation_history_, kJointPosRelOffset,
                            policy_observation::kFrameStack, current_terms.joint_pos_rel);
        append_term_history(observation_history_, kJointVelRelOffset,
                            policy_observation::kFrameStack, current_terms.joint_vel_rel);
        append_term_history(observation_history_, kLastActionOffset,
                            policy_observation::kFrameStack, current_terms.last_action);
    }

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
        if (!index_in_range(motor_index, kDof)) {
            error = "joint mapping missing direct motor for model index " +
                    std::to_string(model_index);
            return false;
        }

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
    AnkleFkState& state,
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
        state.solver.solve(upper_motor, lower_motor);
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
