#include "robot/action_processor.hpp"

#include "base/tool.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"
#include "robot/robot_motor_session.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

namespace inference::robot_detail {

using robot_base::index_in_range;

namespace {

bool finite_feedback(const motor_base::RealtimeMotorFeedback& feedback,
                     std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(feedback.q[i]) ||
            !std::isfinite(feedback.dq[i]) ||
            !std::isfinite(feedback.torque_percent[i])) {
            return false;
        }
    }
    return true;
}

// filter_cutoff/filter_dt 已由 validate_policy_config 校验为有限正值，这里只做纯计算。
LowPass2Coefficients compute_low_pass_coefficients(
    const AnkleTorqueControlConfig& config)
{
    const double ts = config.filter_dt_s;
    const double wc = config.filter_cutoff_rad_s;
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

double clamp_symmetric(double value, double limit)
{
    return std::max(-limit, std::min(limit, value));
}

std::string ankle_limit_error(const char* ankle_name,
                              const char* axis_name,
                              int model_dof,
                              double current,
                              double lower,
                              double upper)
{
    return std::string(ankle_name) + " " + axis_name +
           " dof " + std::to_string(model_dof) +
           " exceeded hard limit: current=" + std::to_string(current) +
           ", range=[" + std::to_string(lower) +
           ", " + std::to_string(upper) + "]";
}

}  // namespace

ActionProcessor::ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                                 PolicyConfig policy_config,
                                 AnkleTorqueControlConfig torque_config)
    : mapping_(std::move(mapping)),
      policy_config_(std::move(policy_config)),
      low_pass_coeffs_(compute_low_pass_coefficients(torque_config))
{
    reset_runtime_state();
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

    auto reset_torque_state = [this](const AnkleParallelMap& ankle_map,
                                     AnkleTorqueState& state) {
        double pitch = 0.0;
        double roll = 0.0;
        const int count = dof_count();
        if (policy_config_.stand_pose_rad.size() == static_cast<std::size_t>(count) &&
            index_in_range(ankle_map.model_pitch_dof, count) &&
            index_in_range(ankle_map.model_roll_dof, count)) {
            pitch = policy_config_.stand_pose_rad[static_cast<std::size_t>(ankle_map.model_pitch_dof)];
            roll = policy_config_.stand_pose_rad[static_cast<std::size_t>(ankle_map.model_roll_dof)];
        }
        state.reset(roll, pitch);
    };

    if (mapping_ && mapping_->configured()) {
        reset_torque_state(mapping_->left_ankle(), left_ankle_torque_);
        reset_torque_state(mapping_->right_ankle(), right_ankle_torque_);
    } else {
        left_ankle_torque_.reset();
        right_ankle_torque_.reset();
    }
}


bool ActionProcessor::build_motor_targets(
    const std::vector<double>& target_q_model_rad,  // 模型计算出的关节目标角
    std::vector<double>&       target_motor_rad,    // 电机目标角(引用)
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

bool ActionProcessor::apply_ankle_ik(
    const std::vector<double>& target_q_model_rad,  // 模型计算出的关节目标角
    std::vector<double>&       target_motor_rad,    // 电机目标角(引用)
    const AnkleParallelMap&    ankle_map,           // 脚踝关节的映射关系
    AnkleIkState&              state,               // 脚踝IK求解器的状态
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
    double roll  = target_q_model_rad[static_cast<std::size_t>(ankle_map.model_roll_dof)];

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
        roll  = std::max(roll_lo,  std::min(roll_hi, roll));
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

void ActionProcessor::LowPass2State::reset()
{
    x1 = 0.0;
    x2 = 0.0;
    y1 = 0.0;
    y2 = 0.0;
}

void ActionProcessor::AnkleTorqueState::reset(double roll, double pitch)
{
    fk_solver.reset(roll, pitch);
    pitch_filter.reset();
    roll_filter.reset();
}

bool ActionProcessor::build_policy_impedance_command(
    const FixedModelTarget& target_q_model_rad,
    const motor_base::RealtimeMotorFeedback& motor_feedback,
    const std::vector<double>& motor_kp,
    const std::vector<double>& motor_kd,
    const AnkleTorqueControlConfig& torque_config,
    FixedPolicyMotorCommand& command,
    std::string& error)
{
    const int count = dof_count();
    if (motor_feedback.motor_count != static_cast<std::size_t>(count)) {
        error = "policy impedance command size mismatch";
        return false;
    }
    if (!finite_feedback(motor_feedback, static_cast<std::size_t>(count))) {
        error = "policy impedance command inputs must be finite";
        return false;
    }

    command = FixedPolicyMotorCommand{};
    command.setpoint_count = static_cast<std::size_t>(count);

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

        const auto motor_slot = static_cast<std::size_t>(motor_index);
        const double motor_target =
            mapping_->direction_for_motor(motor_index) * q;
        command.setpoints[motor_slot] =
            motor_base::ImpedanceSetpoint(motor_target,
                                          0.0,
                                          0.0,
                                          motor_kp[motor_slot],
                                          motor_kd[motor_slot]);
    }

    if (!apply_ankle_torque_control(target_q_model_rad,
                                    motor_feedback,
                                    "left ankle",
                                    mapping_->left_ankle(),
                                    torque_config,
                                    left_ankle_torque_,
                                    command,
                                    error)) {
        return false;
    }
    if (!apply_ankle_torque_control(target_q_model_rad,
                                    motor_feedback,
                                    "right ankle",
                                    mapping_->right_ankle(),
                                    torque_config,
                                    right_ankle_torque_,
                                    command,
                                    error)) {
        return false;
    }

    error.clear();
    return true;
}

bool ActionProcessor::apply_ankle_torque_control(
    const FixedModelTarget& target_q_model_rad,
    const motor_base::RealtimeMotorFeedback& motor_feedback,
    const char* ankle_name,
    const AnkleParallelMap& ankle_map,
    const AnkleTorqueControlConfig& torque_config,
    AnkleTorqueState& state,
    FixedPolicyMotorCommand& command,
    std::string& error)
{
    const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
    const auto roll_index = static_cast<std::size_t>(ankle_map.model_roll_dof);
    const auto upper_index = static_cast<std::size_t>(ankle_map.upper_motor_index);
    const auto lower_index = static_cast<std::size_t>(ankle_map.lower_motor_index);

    double desired_pitch = target_q_model_rad[pitch_index];
    double desired_roll = target_q_model_rad[roll_index];
    if (has_relative_limits()) {
        const double pitch_lo =
            policy_config_.stand_pose_rad[pitch_index] + policy_config_.joint_min_rad[pitch_index];
        const double pitch_hi =
            policy_config_.stand_pose_rad[pitch_index] + policy_config_.joint_max_rad[pitch_index];
        const double roll_lo =
            policy_config_.stand_pose_rad[roll_index] + policy_config_.joint_min_rad[roll_index];
        const double roll_hi =
            policy_config_.stand_pose_rad[roll_index] + policy_config_.joint_max_rad[roll_index];
        desired_pitch = std::max(pitch_lo, std::min(pitch_hi, desired_pitch));
        desired_roll = std::max(roll_lo, std::min(roll_hi, desired_roll));
    }

    const int upper_direction = mapping_->direction_for_motor(ankle_map.upper_motor_index);
    const int lower_direction = mapping_->direction_for_motor(ankle_map.lower_motor_index);
    const double upper_motor = upper_direction * motor_feedback.q[upper_index];
    const double lower_motor = lower_direction * motor_feedback.q[lower_index];
    const double upper_motor_velocity =
        upper_direction * motor_feedback.dq[upper_index];
    const double lower_motor_velocity =
        lower_direction * motor_feedback.dq[lower_index];

    const ankle_motor_fk::FootAngles foot =
        state.fk_solver.solve(upper_motor, lower_motor);
    if (!foot.reachable ||
        !std::isfinite(foot.pitch) ||
        !std::isfinite(foot.roll)) {
        error = "ankle FK failed while building policy impedance command";
        return false;
    }

    const double pitch_lower =
        policy_config_.stand_pose_rad[pitch_index] + policy_config_.joint_min_rad[pitch_index];
    const double pitch_upper =
        policy_config_.stand_pose_rad[pitch_index] + policy_config_.joint_max_rad[pitch_index];
    const double roll_lower =
        policy_config_.stand_pose_rad[roll_index] + policy_config_.joint_min_rad[roll_index];
    const double roll_upper =
        policy_config_.stand_pose_rad[roll_index] + policy_config_.joint_max_rad[roll_index];
    if (foot.pitch < pitch_lower || foot.pitch > pitch_upper) {
        error = ankle_limit_error(ankle_name,
                                  "pitch",
                                  ankle_map.model_pitch_dof,
                                  foot.pitch,
                                  pitch_lower,
                                  pitch_upper);
        return false;
    }
    if (foot.roll < roll_lower || foot.roll > roll_upper) {
        error = ankle_limit_error(ankle_name,
                                  "roll",
                                  ankle_map.model_roll_dof,
                                  foot.roll,
                                  roll_lower,
                                  roll_upper);
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
        torque_config.virtual_kp[0] * (desired_pitch - foot.pitch) -
        torque_config.virtual_kd[0] * pitch_velocity;
    const double roll_torque_des =
        torque_config.virtual_kp[1] * (desired_roll - foot.roll) -
        torque_config.virtual_kd[1] * roll_velocity;

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

    const double scale = 1000.0 / torque_config.motor_rated_torque_nm;
    const double upper_effort_permille = clamp_symmetric(
        upper_direction * upper_torque_nm * scale,
        torque_config.target_torque_limit_permille);
    const double lower_effort_permille = clamp_symmetric(
        lower_direction * lower_torque_nm * scale,
        torque_config.target_torque_limit_permille);

    command.setpoints[upper_index] =
        motor_base::ImpedanceSetpoint(motor_feedback.q[upper_index],
                                      0.0,
                                      upper_effort_permille,
                                      0.0,
                                      0.0);
    command.setpoints[lower_index] =
        motor_base::ImpedanceSetpoint(motor_feedback.q[lower_index],
                                      0.0,
                                      lower_effort_permille,
                                      0.0,
                                      0.0);
    command.target_effort_permille[upper_index] = upper_effort_permille;
    command.target_effort_permille[lower_index] = lower_effort_permille;

    error.clear();
    return true;
}

int ActionProcessor::dof_count() const noexcept
{
    return mapping_ ? mapping_->dof_count() : 0;
}

bool ActionProcessor::has_relative_limits() const
{
    const auto count = static_cast<std::size_t>(dof_count());
    return policy_config_.stand_pose_rad.size() == count &&
           policy_config_.joint_min_rad.size()  == count &&
           policy_config_.joint_max_rad.size()  == count;
}



bool ActionProcessor::build_reset_start_model_pose(
    const std::vector<double>& current_motor_q, // 当前电机真实角度(rad)
    const std::vector<double>& target_model_q,  // 期望的模型关节角(作为fk初始值)
    std::vector<double>&       start_model_q,   // 输出: 当前的模型关节角(rad)
    std::string& error) const
{
    if (!mapping_) {
        error = "joint mapping is not configured";
        return false;
    }

    const int count = dof_count();
    if (static_cast<int>(current_motor_q.size()) != count ||
        static_cast<int>(target_model_q.size())  != count) {
        error = "reset pose size mismatch";
        return false;
    }

    start_model_q.assign(static_cast<std::size_t>(count), 0.0);

    // 按照模型顺序遍历，将电机角度转换为模型关节角度
    for (int model_index = 0; model_index < count; ++model_index) {
        // 跳过脚踝关节的模型 DOF，它们的角度由 IK 求解器计算得出
        if (mapping_->is_parallel_model_dof(model_index)) {
            continue;
        }

        // 得到电机索引
        const int motor_index = mapping_->direct_motor_for_model_dof(model_index);
        if (!index_in_range(motor_index, count)) {
            error = "joint mapping missing direct motor for model index " +
                    std::to_string(model_index);
            return false;
        }
        // 从电机角度转换为模型关节角度，考虑电机方向
        start_model_q[static_cast<std::size_t>(model_index)] =
            mapping_->direction_for_motor(motor_index) * current_motor_q[static_cast<std::size_t>(motor_index)];
    }

    auto fill_ankle_start_model_q = [&](const AnkleParallelMap& ankle_map) {
        const double upper_motor =
            mapping_->direction_for_motor(ankle_map.upper_motor_index) *
            current_motor_q[static_cast<std::size_t>(ankle_map.upper_motor_index)];
        const double lower_motor =
            mapping_->direction_for_motor(ankle_map.lower_motor_index) *
            current_motor_q[static_cast<std::size_t>(ankle_map.lower_motor_index)];

        const auto pitch_index = static_cast<std::size_t>(ankle_map.model_pitch_dof);
        const auto roll_index  = static_cast<std::size_t>(ankle_map.model_roll_dof);
        const double initial_pitch = target_model_q[pitch_index];
        const double initial_roll  = target_model_q[roll_index];
        const ankle_motor_fk::FootAngles foot =
            ankle_motor_fk::solve(upper_motor, lower_motor, initial_roll, initial_pitch);

        if (foot.reachable) {
            start_model_q[pitch_index] = foot.pitch;
            start_model_q[roll_index] = foot.roll;
        } else {
            start_model_q[pitch_index] = initial_pitch;
            start_model_q[roll_index] = initial_roll;
        }
        return true;
    };

    fill_ankle_start_model_q(mapping_->left_ankle());
    fill_ankle_start_model_q(mapping_->right_ankle());
    error.clear();
    return true;
}

}  // namespace inference::robot_detail

