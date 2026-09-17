#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_jacobian.hpp"
#include "policy/policy_observation_config.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace inference::robot_detail {

class ObservationBuilder {
public:
    static constexpr std::size_t kDof = policy_observation::kDof;
    using MotorStateArray = std::array<double, kDof>;
    using JointTermArray  = std::array<float,  kDof>;

    // mapping 由 JointMapping::create() 成功构造，运行期不可变。
    ObservationBuilder(std::shared_ptr<const JointMapping> mapping,
                       ObservationScaleConfig scales,
                       std::array<double, policy_observation::kDof> default_joint_pos_rad,
                       PolicyRuntimeConfig policy_config);

                       
    void reset_runtime_state();
    void commit_policy_action(const PolicyAction& raw_action) noexcept;
    void advance_frame() noexcept;
    std::uint64_t frame_index() const noexcept { return frame_index_; }


    bool build(const MotorStateSnapshot& motor_state,
               const AhrsStateSnapshot& ahrs_state,
               const std::array<double, 3>& target_velocity,
               PolicyObservation& observation,
               std::string& error);

private:
    ankle_motor_fk::Solver left_ankle_fk_;
    ankle_motor_fk::Solver right_ankle_fk_;


    void reset_ankle_state(const AnkleParallelMap& ankle_map,
                                 ankle_motor_fk::Solver& state);
                        
    bool fill_joint_terms(const MotorStateArray& q_motor_rad,
                          const MotorStateArray& dq_motor_rad_s,
                          JointTermArray& joint_pos_rel,
                          JointTermArray& joint_vel_rel,
                          std::string& error);

                                 
    bool fill_ankle_fk_joint_terms(const MotorStateArray& q_motor_rad,
                                   const MotorStateArray& dq_motor_rad_s,
                                   const AnkleParallelMap& ankle_map,
                                   ankle_motor_fk::Solver& state,
                                   JointTermArray& joint_pos_rel,
                                   JointTermArray& joint_vel_rel,
                                   std::string& error) const;

    std::shared_ptr<const JointMapping> mapping_;
    ObservationScaleConfig scales_;
    std::array<double, policy_observation::kDof> default_joint_pos_rad_;
    PolicyRuntimeConfig policy_config_;
    PolicyAction last_action_raw_{};
    PolicyObservation observation_history_{};
    bool observation_history_ready_{false};
    std::uint64_t frame_index_{0};
};

}  // namespace inference::robot_detail
