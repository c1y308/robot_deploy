#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace inference {

struct MotorStateSnapshot;
struct AhrsStateSnapshot;

struct ObservationTerms {
    std::array<float, policy_observation::kBaseAngVelSize> base_ang_vel{};
    std::array<float, policy_observation::kProjectedGravitySize> projected_gravity{};
    std::array<float, policy_observation::kVelocityCommandsSize> velocity_commands{};
    std::array<float, policy_observation::kJointPosRelSize> joint_pos_rel{};
    std::array<float, policy_observation::kJointVelRelSize> joint_vel_rel{};
    std::array<float, kDof> last_action{};
};

class ObservationBuilder {
public:
    // config 和已构造的 mapping 在运行期不可变，且必须比本对象存活更久。
    ObservationBuilder(const RobotInterfaceConfig& config,
                       const JointMapping& mapping);

    // 保存上一动作，并将各分量裁剪到 [-1, 1]。
    void set_last_action(const std::array<float, kDof>& raw_action) noexcept;

    bool build(const MotorStateSnapshot&    motor_state,
            const    AhrsStateSnapshot&     ahrs_state,
            const    std::array<double, 3>& target_velocity,
            PolicyObservation&              observation,
            std::string& error);

    void          advance_frame() noexcept;
    std::uint64_t frame_index() const noexcept { return frame_index_; }


private:
    using MotorStateArray = std::array<double, motor_base::kMotorNum>;
    using JointTermArray  = std::array<float, kDof>;

    std::array<float, policy_observation::kGaitPhaseSize>
    build_gait_phase(
        const std::array<float, policy_observation::kVelocityCommandsSize>& velocity_commands) const;

    template <std::size_t TermSize>
    void update_stacked_term(std::size_t& offset,
                             const std::array<float, TermSize>& current_term);

    bool build_joint_terms(const MotorStateArray& q_motor_rad,      // 位置
                          const MotorStateArray& dq_motor_rad_s,    // 速度
                          JointTermArray& joint_pos_rel,
                          JointTermArray& joint_vel_rel,
                          std::string& error);

    bool build_ankle_joint_terms(const MotorStateArray& q_motor_rad,
                                  const MotorStateArray&  dq_motor_rad_s,
                                  const AnkleMap& ankle_map,
                                  ankle_motor_fk::Solver& state,
                                  JointTermArray& joint_pos_rel,
                                  JointTermArray& joint_vel_rel,
                                  std::string& error) const;
    
    // 保存关节映射
    const RobotInterfaceConfig& config_;
    const JointMapping&         mapping_;

    // 运动学正解求解器
    ankle_motor_fk::Solver left_ankle_fk_;
    ankle_motor_fk::Solver right_ankle_fk_;

    std::array<float, kDof> last_action_{};                // 需要裁剪到 [-1, 1] 的无量纲历史动作。
    PolicyObservation stacked_observation_{};   // 拼接完成的帧
    bool observation_history_ready_{false};     // 是否是首帧
    std::uint64_t frame_index_{0};
};

}  // namespace inference
