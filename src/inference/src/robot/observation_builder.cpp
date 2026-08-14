#include "robot/observation_builder.hpp"

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
}

// 从policy_config_中获取stand_pose_rad的roll和pitch值，调用ankle_fk的reset函数，重置ankle_fk
void ObservationBuilder::reset_ankle_state(const AnkleParallelMap& ankle_map,
                                                 AnkleFkState&     state)
{
    double pitch = 0.0;
    double roll  = 0.0;
    if (policy_config_.stand_pose_rad.size() == kDof &&
        index_in_range(ankle_map.model_pitch_dof, static_cast<int>(kDof)) &&
        index_in_range(ankle_map.model_roll_dof, static_cast<int>(kDof)))
    {
        pitch = policy_config_.stand_pose_rad[ankle_map.model_pitch_dof];
        roll  = policy_config_.stand_pose_rad[ankle_map.model_roll_dof];
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
    const ImuStateSnapshot&      imu_state,         // IMU快照
    const std::array<double, 3>& target_velocity,   // 目标速度
    const PolicyAction&          last_action,       // 上一次的策略动作(array<float, kDof>)
    PolicyObservationTerms&      terms,             // 当前观测帧的引用(array构成的结构体)
    std::string& error)
{
    if (motor_state.position_rad.size() != kDof || motor_state.velocity_rad_s.size() != kDof) {
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
    if (!finite_vector(motor_state.position_rad) || !finite_vector(motor_state.velocity_rad_s) ||
        !finite_array3(imu_state.body_ang_vel)   || !finite_array3(imu_state.projected_gravity)) {
        error = "observation source value is not finite";
        return false;
    }

    // 使用当前数据构建下一观测帧
    PolicyObservationTerms current_terms;
    current_terms.velocity_commands = {
        static_cast<float>(target_velocity[0] * policy_config_.command_scale[0]),
        static_cast<float>(target_velocity[1] * policy_config_.command_scale[1]),
        static_cast<float>(target_velocity[2] * policy_config_.command_scale[2])
    };
    for (int i = 0; i < 3; ++i) {
        current_terms.base_ang_vel[i] = static_cast<float>(
            imu_state.body_ang_vel[i] * policy_config_.body_ang_vel_scale[i]);
        current_terms.projected_gravity[i] =
            static_cast<float>(imu_state.projected_gravity[i]);
    }
    current_terms.last_action = last_action;


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

    terms = current_terms;
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
    if (!mapping_ || !mapping_->configured()) {
        error = "joint mapping is not configured";
        return false;
    }
    if (mapping_->dof_count() != static_cast<int>(kDof)) {
        error = "joint mapping DOF count does not match policy DOF";
        return false;
    }
    if (policy_config_.stand_pose_rad.size() != kDof ||
        policy_config_.dof_pos_scale.size()  != kDof ||
        policy_config_.dof_vel_scale.size()  != kDof) {
        error = "policy observation vectors must have one value per DOF";
        return false;
    }

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
            static_cast<float>((q_model - policy_config_.stand_pose_rad[model_index]) *
                               policy_config_.dof_pos_scale[model_index]);
        joint_vel_rel[model_index] =
            static_cast<float>(dq_model * policy_config_.dof_vel_scale[model_index]);
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
    if (!index_in_range(ankle_model_pitch_dof, kDof) ||
        !index_in_range(ankle_model_roll_dof, kDof) ||
        !index_in_range(ankle_map.upper_motor_index, kDof) ||
        !index_in_range(ankle_map.lower_motor_index, kDof)) {
        error = "ankle map contains an out-of-range index";
        return false;
    }

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
    const auto roll_index = static_cast<std::size_t>(ankle_model_roll_dof);


    // 计算脚踝关节的相对偏移并缩放
    joint_pos_rel[pitch_index] =
        static_cast<float>((foot.pitch - policy_config_.stand_pose_rad[pitch_index]) *
                           policy_config_.dof_pos_scale[pitch_index]);
    joint_pos_rel[roll_index] =
        static_cast<float>((foot.roll - policy_config_.stand_pose_rad[roll_index]) *
                           policy_config_.dof_pos_scale[roll_index]);


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
        static_cast<float>(pitch_velocity * policy_config_.dof_vel_scale[pitch_index]);
    joint_vel_rel[roll_index] =
        static_cast<float>(roll_velocity * policy_config_.dof_vel_scale[roll_index]);

    error.clear();
    return true;
}

}  // namespace inference::robot_detail
