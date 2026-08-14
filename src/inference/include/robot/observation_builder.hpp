#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "policy/policy_runtime.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_imu_session.hpp"
#include "robot/robot_motor_session.hpp"
#include "robot/robot_config.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

namespace inference::robot_detail {

class ObservationBuilder {
public:
    static constexpr std::size_t kDof = policy_observation::kDof;
    using MotorStateArray = std::array<double, kDof>;
    using JointTermArray  = std::array<float,  kDof>;

    ObservationBuilder(std::shared_ptr<const JointMapping> mapping,
                       PolicyConfig policy_config);

                       
    void reset_runtime_state();


    bool build(const MotorStateSnapshot& motor_state,
               const ImuStateSnapshot& imu_state,
               const std::array<double, 3>& target_velocity,
               const PolicyAction& last_action,
               PolicyObservationTerms& terms,
               std::string& error);

private:
    struct AnkleFkState {
        ankle_motor_fk::Solver solver;

        void reset(double roll = 0.0, double pitch = 0.0);
    };

    AnkleFkState left_ankle_fk_;
    AnkleFkState right_ankle_fk_;


    void reset_ankle_state(const AnkleParallelMap& ankle_map,
                                 AnkleFkState&     state);
                        
    bool fill_joint_terms(const MotorStateArray& q_motor_rad,
                          const MotorStateArray& dq_motor_rad_s,
                          JointTermArray& joint_pos_rel,
                          JointTermArray& joint_vel_rel,
                          std::chrono::steady_clock::time_point now,
                          std::string& error);

                                 
    void fill_ankle_fk_joint_terms(const MotorStateArray& q_motor_rad,
                                   double dt,
                                   bool has_valid_dt,
                                   const AnkleParallelMap& ankle_map,
                                   AnkleFkState& state,
                                   JointTermArray& joint_pos_rel,
                                   JointTermArray& joint_vel_rel) const;

    std::shared_ptr<const JointMapping> mapping_;
    PolicyConfig policy_config_;

    bool ready_ = false;
    std::chrono::steady_clock::time_point last_time_{};
};

}  // namespace inference::robot_detail
