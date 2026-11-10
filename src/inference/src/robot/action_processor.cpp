#include "robot/action_processor.hpp"

#include "kinematics/ankle_jacobian.hpp"
#include "robot/robot_motor_session.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <string>

namespace inference {

ActionProcessor::ActionProcessor(const RobotInterfaceConfig& config,
                                 const JointMapping& mapping)
    : config_(config),
      mapping_(mapping),
      low_pass_coeffs_(compute_lowpass2_coefficients()),
      left_ankle_torque_state_(
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.left_ankle().model_roll_dof)],
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.left_ankle().model_pitch_dof)]),
      right_ankle_torque_state_(
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.right_ankle().model_roll_dof)],
          config.default_joint_pos_rad[
              static_cast<std::size_t>(mapping.right_ankle().model_pitch_dof)])
{
}

// filter_cutoff/filter_dt 已由配置加载器校验为有限正值，这里只做纯计算。
ActionProcessor::LowPass2Coefficients ActionProcessor::compute_lowpass2_coefficients() const
{
    const double ts = config_.ankle.filter_dt_s;
    const double wc = config_.ankle.filter_cutoff_rad_s;
    const double ts2wc2 = ts * ts * wc * wc;
    const double d = 2500.0 * ts2wc2 + 7071.0 * ts * wc + 10000.0;

    LowPass2Coefficients coeffs;
    coeffs.b0 = 2500.0 * ts2wc2 / d;
    coeffs.b1 = 5000.0 * ts2wc2 / d;
    coeffs.b2 = 2500.0 * ts2wc2 / d;
    coeffs.a1 = -(5000.0 * ts2wc2 - 20000.0) / d;
    coeffs.a2 = -(2500.0 * ts2wc2 - 7071.0 * ts * wc + 10000.0) / d;
    return coeffs;
}

// 目标位置和反馈位置共用同一限位顺序，position_at 直接读取各自的数据源。
template <typename PositionAt>
bool ActionProcessor::validate_ankle_motor_limits(
    PositionAt  position_at,
    const char* source,
    std::string& error) const
{
    // 限位配置顺序：[左上、左下、右上、右下]。
    const std::array<int, 4> ankle_motor_indices = {
        mapping_.left_ankle().upper_motor_index,
        mapping_.left_ankle().lower_motor_index,
        mapping_.right_ankle().upper_motor_index,
        mapping_.right_ankle().lower_motor_index
    };
    for (std::size_t i = 0; i < ankle_motor_indices.size(); ++i) {
        const int motor_index = ankle_motor_indices[i];
        const double position_rad = position_at(static_cast<std::size_t>(motor_index));
        const double lower = config_.ankle.motor_pos_min_rad[i];
        const double upper = config_.ankle.motor_pos_max_rad[i];
        if (position_rad < lower || position_rad > upper) {
            error = std::string(source) + " ankle motor " + std::to_string(motor_index) +
                    " exceeded physical limit: position=" + std::to_string(position_rad) +
                    ", range=[" + std::to_string(lower) +
                    ", " + std::to_string(upper) + "]";
            return false;
        }
    }
    return true;
}

// 将模型输出的原始目标数据转换为电机可以直接接受的目标位置
ActionProcessor::ModelJointPositions ActionProcessor::make_model_target(
    const std::array<float, kDof>& raw_action) const noexcept
{
    const auto& config = config_.action;
    ModelJointPositions target;
    for (std::size_t i = 0; i < target.size(); ++i) {
        const double action = static_cast<double>(raw_action[i]);
        /* 进行 raw_action_clip [-1，1]裁剪 */ 
        const double clipped_raw_action = config.raw_action_clip
            ? std::max(-*config.raw_action_clip, std::min(*config.raw_action_clip, action))
            : action;
        /* 先进行 action_scale 缩放 */
        const double scaled_action = clipped_raw_action * config.action_scale[i];
        /* 再进行 action_clip 裁剪 */ 
        const auto& clip = config.action_clip[i];
        target[i] = config_.default_joint_pos_rad[i] +
            std::max(clip[0], std::min(clip[1], scaled_action));
    }
    return target;
}

bool ActionProcessor::build_motor_position_targets(
    const ModelJointPositions&                  target_model_rad,  // 模型顺序的关节目标角
    std::array<double, motor_base::kMotorNum>&  target_motor_rad,  // 电机目标角(引用传递)
    std::string& error)
{
    const int count = mapping_.dof_count();
    target_motor_rad.fill(0.0);

    for (int model_index = 0; model_index < count; ++model_index) {
        
        /* 先判断 model index 是否合法 */
        if (mapping_.is_model_dof(model_index)) {
            continue;
        }
        /* 获取 model index 对应的 motor index */
        const int motor_index = mapping_.get_motor_id(model_index);
        /* 获取 model index 对应的关节角度 */
        double q = target_model_rad[static_cast<std::size_t>(model_index)];

        /* 写入到 motor index 的目标关节角度 数组中 */
        target_motor_rad[static_cast<std::size_t>(motor_index)] =
            mapping_.get_motor_direction(motor_index) * q;
    }

    // 依据 pitch/roll 的模型关节角度，计算脚踝电机的目标角度，写入到 target_motor_rad 对应索引
    if (!ankle_ik(target_model_rad,
                        target_motor_rad,
                        left_ankle_ik_,
                        error)) {
        return false;
    }
    if (!ankle_ik(target_model_rad,
                        target_motor_rad,
                        right_ankle_ik_,
                        error)) {
        return false;
    }

    if (!validate_ankle_motor_limits(
            [&](std::size_t motor_index) { return target_motor_rad[motor_index]; },
            "target", error)) {
        return false;
    }

    error.clear();
    return true;
}

bool ActionProcessor::ankle_ik(
    const ModelJointPositions&                  target_model_rad,   // 模型计算出的关节目标角
    std::array<double, motor_base::kMotorNum>&  target_motor_rad,   // 电机目标角(引用)
    ankle_motor_ik::Solver&                     solver,             // 脚踝IK求解器及其历史状态
    std::string& error)
{
    const bool      is_left    = solver.side() == ankle_motor_ik::AnkleSide::Left;
    const AnkleMap& ankle_map  = is_left ? mapping_.left_ankle() : mapping_.right_ankle();
    const char*     ankle_name = is_left ? "left" : "right";
    /* 依据 ankle_map 中的 pitch/roll 的模型索引 获取 target_model_rad 中存储的目标关节角 */
    const double pitch = target_model_rad[static_cast<std::size_t>(ankle_map.model_pitch_dof)];
    const double roll  = target_model_rad[static_cast<std::size_t>(ankle_map.model_roll_dof)];

    // 创建 candidate_solver 是为了先尝试求解，两个电机都可达后，再更新原求解器的历史状态
    ankle_motor_ik::Solver candidate_solver = solver;
    const ankle_motor_ik::MotorAngles result = candidate_solver.solve(roll, pitch);

    double upper_motor = 0.0;
    double lower_motor = 0.0;

    // 可达后再更新原求解器

    // 本次成功：使用新解，并更新历史
    if (result.reachable()) {
        upper_motor = result.motor1;
        lower_motor = result.motor2;
        solver = candidate_solver;
    } 
    // 本次失败，但有历史有效解：保持上一次目标
    else if (solver.has_solution()) {
        upper_motor = solver.previous_motor1();
        lower_motor = solver.previous_motor2();
    } 
    // 本次失败，且没有历史有效解：报错
    else {
        error = std::string(ankle_name) +
                " ankle IK unreachable before any valid solution: roll=" +
                std::to_string(roll) + " rad, pitch=" + std::to_string(pitch) +
                " rad, motor1_reachable=" +
                (result.motor1_reachable ? "true" : "false") +
                ", motor2_reachable=" +
                (result.motor2_reachable ? "true" : "false");
        return false;
    }

    /* 依据电机索引判断电机角度是否需要反向，然后将电机目标角度写入 target_motor_rad */
    const int upper_motor_direction =
        mapping_.get_motor_direction(ankle_map.upper_motor_index);
    const int lower_motor_direction =
        mapping_.get_motor_direction(ankle_map.lower_motor_index);
    const double upper_motor_target = upper_motor_direction * upper_motor;
    const double lower_motor_target = lower_motor_direction * lower_motor;

    target_motor_rad[static_cast<std::size_t>(ankle_map.upper_motor_index)] =
        upper_motor_target;
    target_motor_rad[static_cast<std::size_t>(ankle_map.lower_motor_index)] =
        lower_motor_target;
    error.clear();
    return true;
}


bool ActionProcessor::build_policy_motor_command(
    const ModelJointPositions&  target_model_rad,
    const MotorFeedback&        motor_feedback,
    PolicyMotorCommand&         command,
    std::string& error)
{
    const int count = mapping_.dof_count();

    // 检查所有受控电机是否可以接受控制命令。
    for (int motor_index = 0; motor_index < count; ++motor_index) {
        const auto& motor =
            motor_feedback[static_cast<std::size_t>(motor_index)];
        if (!motor.control_ready) {
            error = "motor feedback is not control-ready at index " +
                    std::to_string(motor_index) +
                    ": comm_ok=" + (motor.comm_ok ? "true" : "false") +
                    ", enabled=" + (motor.enabled ? "true" : "false") +
                    ", faulted=" + (motor.faulted ? "true" : "false") +
                    ", control_ready=" +
                    (motor.control_ready ? "true" : "false");
            return false;
        }
    }
    if (!validate_ankle_motor_limits(
            [&](std::size_t motor_index) { return motor_feedback[motor_index].position_rad; },
            "feedback", error)) {
        return false;
    }

    command = PolicyMotorCommand{};
    command.setpoint_count = static_cast<std::size_t>(count);

    // 按照 model_index 顺序遍历目标关节角。跳过脚踝模型 DOF
    for (int model_index = 0; model_index < count; ++model_index) {

        if (mapping_.is_model_dof(model_index)) {
            continue;
        }

        const int motor_index = mapping_.get_motor_id(model_index);
        const double q = target_model_rad[static_cast<std::size_t>(model_index)];

        const auto motor_slot = static_cast<std::size_t>(motor_index);
        const double motor_target = mapping_.get_motor_direction(motor_index) * q;
        command.setpoints[motor_slot] =
            motor_base::ImpedanceSetpoint(motor_target,
                                          0.0,
                                          0.0,
                                          config_.motor.mit_kp[motor_slot],
                                          config_.motor.mit_kd[motor_slot]);
    }

    if (!build_ankle_torque_command(ankle_motor_ik::AnkleSide::Left,
                                  target_model_rad, motor_feedback, command, error)) {
        return false;
    }
    if (!build_ankle_torque_command(ankle_motor_ik::AnkleSide::Right,
                                  target_model_rad, motor_feedback, command, error)) {
        return false;
    }

    error.clear();
    return true;
}

bool ActionProcessor::build_ankle_torque_command(
    ankle_motor_ik::AnkleSide  side,
    const ModelJointPositions& target_model_rad,
    const MotorFeedback&       motor_feedback,
    PolicyMotorCommand&        command,
    std::string& error)
{
    const bool is_left = side == ankle_motor_ik::AnkleSide::Left;
    const auto& ankle_map = is_left ? mapping_.left_ankle() : mapping_.right_ankle();
    auto& state = is_left ? left_ankle_torque_state_ : right_ankle_torque_state_;

    const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
    const auto roll_index  = static_cast<std::size_t>(ankle_map.model_roll_dof);
    const auto upper_index = static_cast<std::size_t>(ankle_map.upper_motor_index);
    const auto lower_index = static_cast<std::size_t>(ankle_map.lower_motor_index);

    const double desired_pitch = target_model_rad[pitch_index];
    const double desired_roll = target_model_rad[roll_index];

    const int upper_direction = mapping_.get_motor_direction(ankle_map.upper_motor_index);
    const int lower_direction = mapping_.get_motor_direction(ankle_map.lower_motor_index);
    const double upper_position_rad = motor_feedback[upper_index].position_rad;
    const double lower_position_rad = motor_feedback[lower_index].position_rad;
    const double upper_motor = upper_direction * upper_position_rad;
    const double lower_motor = lower_direction * lower_position_rad;
    const double upper_motor_velocity =
        upper_direction * motor_feedback[upper_index].velocity_rad_s;
    const double lower_motor_velocity =
        lower_direction * motor_feedback[lower_index].velocity_rad_s;

    const ankle_motor_fk::FootAngles foot =
        state.fk_solver.solve(upper_motor, lower_motor);
    if (!foot.reachable ||
        !std::isfinite(foot.pitch) ||
        !std::isfinite(foot.roll)) {
        error = "ankle FK failed while building policy impedance command";
        return false;
    }

    ankle_jacobian::Result jacobian;
    std::string jacobian_error;
    if (!ankle_jacobian::solve(foot.pitch,
                                     foot.roll,
                                     upper_motor,
                                     lower_motor,
                                     jacobian,
                                     jacobian_error)) {
        error = "failed to build ankle torque jacobian: " + jacobian_error;
        return false;
    }

    const double pitch_velocity =
        jacobian.virtual_from_motor[0][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[0][1] * lower_motor_velocity;
    const double roll_velocity =
        jacobian.virtual_from_motor[1][0] * upper_motor_velocity +
        jacobian.virtual_from_motor[1][1] * lower_motor_velocity;

    const double pitch_torque_des =
        config_.ankle.virtual_kp[0] * (desired_pitch - foot.pitch) -
        config_.ankle.virtual_kd[0] * pitch_velocity;
    const double roll_torque_des =
        config_.ankle.virtual_kp[1] * (desired_roll - foot.roll) -
        config_.ankle.virtual_kd[1] * roll_velocity;

    auto apply_low_pass = [this](double input, LowPass2State& filter_state) {
        const double output =
            low_pass_coeffs_.b0 * input +
            low_pass_coeffs_.b1 * filter_state.x1 +
            low_pass_coeffs_.b2 * filter_state.x2 +
            low_pass_coeffs_.a1 * filter_state.y1 +
            low_pass_coeffs_.a2 * filter_state.y2;

        filter_state.x2 = filter_state.x1;
        filter_state.x1 = input;
        filter_state.y2 = filter_state.y1;
        filter_state.y1 = output;
        return output;
    };

    const double pitch_torque_lp =
        apply_low_pass(pitch_torque_des, state.pitch_filter);
    const double roll_torque_lp =
        apply_low_pass(roll_torque_des, state.roll_filter);

    const double upper_torque_nm =
        jacobian.virtual_from_motor[0][0] * pitch_torque_lp +
        jacobian.virtual_from_motor[1][0] * roll_torque_lp;
    const double lower_torque_nm =
        jacobian.virtual_from_motor[0][1] * pitch_torque_lp +
        jacobian.virtual_from_motor[1][1] * roll_torque_lp;

    const double scale = 1000.0 / config_.ankle.motor_rated_torque_nm;
    const double upper_effort_permille = std::max(
        -config_.ankle.target_torque_limit_permille,
        std::min(config_.ankle.target_torque_limit_permille,
                 upper_direction * upper_torque_nm * scale));
    const double lower_effort_permille = std::max(
        -config_.ankle.target_torque_limit_permille,
        std::min(config_.ankle.target_torque_limit_permille,
                 lower_direction * lower_torque_nm * scale));

    command.setpoints[upper_index] =
        motor_base::ImpedanceSetpoint(upper_position_rad,
                                      0.0,
                                      upper_effort_permille,
                                      0.0,
                                      0.0);
    command.setpoints[lower_index] =
        motor_base::ImpedanceSetpoint(lower_position_rad,
                                      0.0,
                                      lower_effort_permille,
                                      0.0,
                                      0.0);

    error.clear();
    return true;
}

}  // namespace inference
