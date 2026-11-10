#include "robot/observation_builder.hpp"

#include "kinematics/ankle_jacobian.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "tool/tool.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace inference {

using robot_base::finite_array;

ObservationBuilder::ObservationBuilder(const RobotInterfaceConfig& config,
                                       const JointMapping& mapping)
    : config_(config),
      mapping_(mapping),
      left_ankle_fk_(
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.left_ankle().model_roll_dof)],
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.left_ankle().model_pitch_dof)]),
      right_ankle_fk_(
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.right_ankle().model_roll_dof)],
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.right_ankle().model_pitch_dof)])
{
}

void ObservationBuilder::set_last_action(const std::array<float, kDof>& raw_action) noexcept
{
    for (std::size_t i = 0; i < raw_action.size(); ++i) {
        last_action_[i] = std::clamp(raw_action[i], -1.0F, 1.0F);
    }
}

void ObservationBuilder::advance_frame() noexcept
{
    ++frame_index_;
}

// 构建当前推理轮次的观测帧
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
        !finite_array(ahrs_state.body_ang_vel)  || !finite_array(ahrs_state.projected_gravity)) {
        error = "observation source value is not finite";
        return false;
    }

    // 构建当前推理轮次的观测帧
    ObservationTerms current_terms;
    // 构建 机身姿态和速度 项
    for (int i = 0; i < 3; ++i) {
        current_terms.base_ang_vel[i] = static_cast<float>(
            ahrs_state.body_ang_vel[i] * config_.observation_scales.body_ang_vel_scale[i]);
        current_terms.projected_gravity[i] =
            static_cast<float>(ahrs_state.projected_gravity[i] *
                               config_.observation_scales.projected_gravity_scale[i]);
    }
    // 构建 控制指令 项
    current_terms.velocity_commands = {
        static_cast<float>(target_velocity[0] * config_.observation_scales.command_scale[0]),
        static_cast<float>(target_velocity[1] * config_.observation_scales.command_scale[1]),
        static_cast<float>(target_velocity[2] * config_.observation_scales.command_scale[2])
    };
    // 构建 关节相对偏移和速度 项
    if (!build_joint_terms(motor_state.position_rad,
                          motor_state.velocity_rad_s,
                          current_terms.joint_pos_rel,
                          current_terms.joint_vel_rel,
                          error)) {
        return false;
    }
    // 构建 历史动作 项
    for (std::size_t i = 0; i < last_action_.size(); ++i) {
        current_terms.last_action[i] =
            static_cast<float>(last_action_[i] * config_.observation_scales.last_action_scale[i]);
    }

    // 按模型要求的观测项顺序更新整体观测数组（由多个帧拼接而成）
    std::size_t offset = 0;
    update_stacked_term(offset, current_terms.base_ang_vel);
    update_stacked_term(offset, current_terms.projected_gravity);
    update_stacked_term(offset, current_terms.velocity_commands);

    // 如果开启了步态相位，则还需要构建步态相位
    if (config_.policy.gait.enabled) {
        auto gait_phase = build_gait_phase(current_terms.velocity_commands);
        for (std::size_t i = 0; i < gait_phase.size(); ++i) {
            gait_phase[i] = static_cast<float>(gait_phase[i] * config_.observation_scales.gait_phase_scale[i]);
        }
        update_stacked_term(offset, gait_phase);
    }

    update_stacked_term(offset, current_terms.joint_pos_rel);
    update_stacked_term(offset, current_terms.joint_vel_rel);
    update_stacked_term(offset, current_terms.last_action);
    // 首此用首帧填充所有
    observation_history_ready_ = true;

    observation = stacked_observation_;
    error.clear();
    return true;
}

// 填充此帧的关节数据
bool ObservationBuilder::build_joint_terms(
    const MotorStateArray& q_motor_rad,
    const MotorStateArray& dq_motor_rad_s, // 输入物理顺序的电机位置和速度
    JointTermArray& joint_pos_rel,         
    JointTermArray& joint_vel_rel,         // 输出模型顺序的关节的相对偏移和速度
    std::string& error)
{
    // 按照模型顺序遍历关节，因为观测是通过模型顺序排布
    for (int model_index = 0; model_index < kDof; ++model_index) {

        // 如果是脚踝模型的关节，则跳过，脚踝通过 build_ankle_joint_terms 构建
        if (mapping_.is_model_dof(model_index)) {
            continue;
        }

        // 得到模型 id 对应的电机 id
        const int motor_index = mapping_.get_motor_id(model_index);
        // 方向转换为模型方向
        const double direction = static_cast<double>(mapping_.get_motor_direction(motor_index));

        const double q_model   = direction * q_motor_rad[motor_index];
        const double dq_model  = direction * dq_motor_rad_s[motor_index];

        //  计算模型关节的相对偏移和速度，并进行缩放
        joint_pos_rel[model_index] =
            static_cast<float>((q_model - config_.default_joint_pos_rad[model_index]) *
                                config_.observation_scales.dof_pos_scale[model_index]);
        joint_vel_rel[model_index] =
            static_cast<float>(dq_model * config_.observation_scales.dof_vel_scale[model_index]);
    }

    // 针对脚踝模型的关节，使用FK计算位置，并用解析 Jacobian 把电机速度映射为虚拟关节速度。
    if (!build_ankle_joint_terms(   q_motor_rad,
                                    dq_motor_rad_s,
                                    mapping_.left_ankle(),
                                    left_ankle_fk_,
                                    joint_pos_rel,
                                    joint_vel_rel,
                                    error)) {
        return false;
    }
    if (!build_ankle_joint_terms(   q_motor_rad,
                                    dq_motor_rad_s,
                                   mapping_.right_ankle(),
                                   right_ankle_fk_,
                                    joint_pos_rel,
                                    joint_vel_rel,
                                    error)) {
        return false;
    }

    return true;
}

// 构建脚踝的 pitch 和 roll 的相对偏移和速度，使用 FK 和 Jacobian
bool ObservationBuilder::build_ankle_joint_terms(
    const MotorStateArray& q_motor_rad,
    const MotorStateArray& dq_motor_rad_s,
    const AnkleMap& ankle_map,              // 选取本次解算的是左脚踝还是右脚踝
    ankle_motor_fk::Solver& state,
    JointTermArray& joint_pos_rel,
    JointTermArray& joint_vel_rel,
    std::string& error) const
{

    // 首先得到脚踝电机的方向、位置、速度，将电机位置和速度转换为模型方向
    const int upper_motor_direction =
        mapping_.get_motor_direction(ankle_map.upper_motor_index);
    const int lower_motor_direction =
        mapping_.get_motor_direction(ankle_map.lower_motor_index);

    const double upper_motor_position = upper_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)];
    const double lower_motor_position = lower_motor_direction *
        q_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)];
    const double upper_motor_velocity = upper_motor_direction *
        dq_motor_rad_s[static_cast<std::size_t>(ankle_map.upper_motor_index)];
    const double lower_motor_velocity = lower_motor_direction *
        dq_motor_rad_s[static_cast<std::size_t>(ankle_map.lower_motor_index)];
    /****************************************************************************/

    // 正解得到对应的脚踝的 pitch 和 roll 角度
    const ankle_motor_fk::FootAngles foot =
        state.solve(upper_motor_position, lower_motor_position);
    if (!foot.reachable ||
        !std::isfinite(foot.pitch) ||
        !std::isfinite(foot.roll)) {
        error = "ankle FK failed while building observation";
        return false;
    }


    // 获取脚踝 pitch 和 roll 在观测数组中的索引
    const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
    const auto roll_index  = static_cast<std::size_t>(ankle_map.model_roll_dof);

    // 计算脚踝关节的相对偏移并缩放，然后写入观测数组
    joint_pos_rel[pitch_index] =
        static_cast<float>((foot.pitch - config_.default_joint_pos_rad[pitch_index]) *
                           config_.observation_scales.dof_pos_scale[pitch_index]);
    joint_pos_rel[roll_index] =
        static_cast<float>((foot.roll - config_.default_joint_pos_rad[roll_index]) *
                           config_.observation_scales.dof_pos_scale[roll_index]);

    // 使用 q_v_dot = J q_m_dot 计算脚踝虚拟关节速度
    ankle_jacobian::Result jacobian;
    std::string jacobian_error;
    if (!ankle_jacobian::solve(foot.pitch,
                                     foot.roll,
                                     upper_motor_position,
                                     lower_motor_position,
                                     jacobian,
                                     jacobian_error)) {
        error = "failed to build ankle jacobian: " + jacobian_error;
        return false;
    }

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
    // 缩放脚踝关节的速度，然后写入观测数组
    joint_vel_rel[pitch_index] =
        static_cast<float>(pitch_velocity * config_.observation_scales.dof_vel_scale[pitch_index]);
    joint_vel_rel[roll_index] =
        static_cast<float>(roll_velocity * config_.observation_scales.dof_vel_scale[roll_index]);

    return true;
}

std::array<float, policy_observation::kGaitPhaseSize>
ObservationBuilder::build_gait_phase(
    const std::array<float, policy_observation::kVelocityCommandsSize>& velocity_commands) const
{
    const auto& gait_config = config_.policy.gait;
    const double command_norm = std::sqrt(
        static_cast<double>(velocity_commands[0]) * velocity_commands[0] +
        static_cast<double>(velocity_commands[1]) * velocity_commands[1] +
        static_cast<double>(velocity_commands[2]) * velocity_commands[2]);

    double gate = std::clamp(
        (command_norm - gait_config.stand_threshold) /
            (gait_config.move_threshold - gait_config.stand_threshold),
        0.0,
        1.0);
    gate = gate * gate * (3.0 - 2.0 * gate);

    constexpr double kTwoPi = 6.28318530717958647692;
    const double global_phase = std::fmod(
        static_cast<double>(frame_index_) * config_.policy.step_dt,
        gait_config.period) / gait_config.period;
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
void ObservationBuilder::update_stacked_term(
    std::size_t& offset, const std::array<float, TermSize>& current_term)
{
    /* 找到当前观测项在历史记录数组中的位置 */
    const auto begin = stacked_observation_.begin() + offset;
    const auto end   = begin + policy_observation::kFrameStack * TermSize;

    /* 已经有历史数据 */
    if (observation_history_ready_) {
        // 左移一次的历史数据
        std::copy(begin + TermSize, end, begin);
        // 填充当前历史数据
        std::copy(current_term.begin(), current_term.end(), end - TermSize);
    } 
    /* 需要同时填充历史数据 */
    else {
        for (auto frame = begin; frame != end; frame += TermSize) {
            std::copy(current_term.begin(), current_term.end(), frame);
        }
    }
    offset += policy_observation::kFrameStack * TermSize;
}


}  // namespace inference
