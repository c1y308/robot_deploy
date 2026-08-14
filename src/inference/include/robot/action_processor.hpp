#pragma once

#include "kinematics/ankle_motor_fk.hpp"
#include "kinematics/ankle_motor_ik.hpp"
#include "robot/joint_mapping.hpp"
#include "robot/robot_config.hpp"

#include <memory>
#include <string>
#include <vector>

namespace inference::robot_detail {

class ActionProcessor {
public:
    ActionProcessor(std::shared_ptr<const JointMapping> mapping,
                    PolicyConfig policy_config);

    void reset_runtime_state();

    bool build_motor_targets(const std::vector<double>& target_q_model_rad,
                             std::vector<double>& target_motor_rad,
                             std::string& error);

    bool build_reset_start_model_pose(const std::vector<double>& current_motor_q,
                                      const std::vector<double>& target_model_q,
                                      std::vector<double>& start_model_q,
                                      std::string& error) const;

private:
    struct AnkleIkState {
        ankle_motor_ik::Solver solver;
        double last_upper_motor = 0.0;
        double last_lower_motor = 0.0;
        bool solved = false;

        void reset();
    };

    int dof_count() const noexcept;
    bool has_relative_limits() const;

    bool apply_ankle_ik(const std::vector<double>& target_q_model_rad,
                        std::vector<double>& target_motor_rad,
                        const AnkleParallelMap& ankle_map,
                        AnkleIkState& state,
                        std::string& error);

    std::shared_ptr<const JointMapping> mapping_;
    PolicyConfig policy_config_;
    AnkleIkState left_ankle_ik_;
    AnkleIkState right_ankle_ik_;
};

}  // namespace inference::robot_detail
